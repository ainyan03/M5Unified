// Fill in here, or pass both from the build (-DWIFI_SSID=\"...\" -DWIFI_PASS=\"...\").
#ifndef WIFI_SSID
#define WIFI_SSID "SET YOUR WIFI SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "SET YOUR WIFI PASS"
#endif


#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>

/// need ESP8266Audio library. ( URL : https://github.com/earlephilhower/ESP8266Audio/ )
#include <AudioOutput.h>
#include <AudioFileSourceICYStream.h>
#include <AudioFileSource.h>
#include <AudioFileSourceBuffer.h>
#include <AudioGeneratorMP3.h>

#include <M5UnitLCD.h>
#include <M5UnitOLED.h>
#include <M5Unified.h>
#include <atomic>

/// set M5Speaker virtual channel (0-7)
static constexpr uint8_t m5spk_virtual_channel = 0;

/// set web radio station url (plain http MP3 streams; a station that cannot
/// be opened is skipped automatically)
static constexpr const char* station_list[][2] =
{
  {"Radio Paradise"    , "http://stream.radioparadise.com/mp3-128"},
  {"FIP"               , "http://icecast.radiofrance.fr/fip-midfi.mp3"},
  {"FIP Jazz"          , "http://icecast.radiofrance.fr/fipjazz-midfi.mp3"},
  {"France Musique"    , "http://icecast.radiofrance.fr/francemusique-midfi.mp3"},
  {"181.fm Beatles"    , "http://listen.181fm.com/181-beatles_128k.mp3"},
  {"181.fm Classical"  , "http://listen.181fm.com/181-classical_128k.mp3"},
  {"181.fm Jazz Mix"   , "http://listen.181fm.com/181-jazzmix_128k.mp3"},
  {"KEXP"              , "http://kexp-mp3-128.streamguys1.com/kexp128.mp3"},
  {"WQXR"              , "http://stream.wqxr.org/wqxr"},
  {"BBC World Service" , "http://stream.live.vc.bbcmedia.co.uk/bbc_world_service"},
  {"Classic FM"        , "http://media-ice.musicradio.com:80/ClassicFMMP3"},
  {"Lite Favorites"    , "http://naxos.cdnstream.com:80/1255_128"},
};
static constexpr const size_t stations = sizeof(station_list) / sizeof(station_list[0]);

class AudioOutputM5Speaker : public AudioOutput
{
  public:
    AudioOutputM5Speaker(m5::Speaker_Class* m5sound, uint8_t virtual_sound_channel = 0)
    {
      _m5sound = m5sound;
      _virtual_ch = virtual_sound_channel;
    }
    virtual ~AudioOutputM5Speaker(void) {};
    /// Call once after M5.begin() and before anything plays on the speaker
    /// (the callback may only be registered before the first request).
    /// Not done in the constructor: at global construction M5 may not exist.
    void setup(void)
    {
      // Buffers are used in rotation and one is refilled only after the
      // speaker task has released it, which it reports through this callback.
      // Two would be enough for that handshake; the third keeps one buffer of
      // slack against decoder and network jitter, as the old three-buffer
      // form had.
      _m5sound->setBufferReleaseCallback(this, bufferReleased);
    }
    virtual bool begin(void) override { return true; }
    virtual bool ConsumeSample(int16_t sample[2]) override
    {
      if (_buffer_index < buf_size)
      {
        _buffer[_index][_buffer_index  ] = sample[0];
        _buffer[_index][_buffer_index+1] = sample[1];
        _buffer_index += 2;

        return true;
      }

      flush();
      return false;
    }
    virtual void flush(void) override
    {
      if (_buffer_index)
      {
        // busy is set before playRaw: the release can only come after the
        // request is queued. playRaw returns false when nothing was queued.
        _busy[_index] = true;
        if (_m5sound->playRaw(_buffer[_index], _buffer_index, hertz, true, 1, _virtual_ch)) { _frames += _buffer_index / 2; }
        else { _busy[_index] = false; }
        if (++_index >= buf_count) { _index = 0; }
        _buffer_index = 0;
        while (_busy[_index]) { vTaskDelay(1); } // wait until the speaker task has released the next buffer
        ++_update_count;
      }
    }
    virtual bool stop(void) override
    {
      // Let the queued buffers play out rather than cutting them: after the
      // last release nothing refers to the buffers any more.
      flush();
      for (size_t i = 0; i < buf_count; ++i)
      {
        while (_busy[i]) { vTaskDelay(1); }
      }
      for (size_t i = 0; i < buf_count; ++i)
      {
        memset(_buffer[i], 0, buf_size * sizeof(int16_t));
      }
      ++_update_count;
      return true;
    }

    /// The most recently queued buffer, for the level/FFT display only: the
    /// producer may start refilling it while it is being read, so a torn
    /// frame is possible (as with any number of buffers when the reader lags).
    const int16_t* getBuffer(void) const { return _buffer[(_index + buf_count - 1) % buf_count]; }
    const uint32_t getUpdateCount(void) const { return _update_count; }
    /// stereo frames the speaker accepted so far, and their rate: what has
    /// actually been played, unlike wall-clock time spent buffering. The
    /// rate is 0 until the decoder has produced its first sample.
    uint32_t getFrames(void) const { return _frames; }
    uint32_t getRate(void) const { return hertz; }

  protected:
    m5::Speaker_Class* _m5sound;
    uint8_t _virtual_ch;
    static constexpr size_t buf_count = 3;
    static constexpr size_t buf_size = 640;
    int16_t _buffer[buf_count][buf_size];
    // These flags are shared between tasks. Use std::atomic<bool> here;
    // plain bool or volatile does not safely share updates between tasks.
    std::atomic<bool> _busy[buf_count] = { {false}, {false}, {false} };
    size_t _buffer_index = 0;
    size_t _index = 0;
    size_t _update_count = 0;
    uint32_t _frames = 0;

    static void bufferReleased(void* args, const void* data, uint8_t)
    {
      auto me = (AudioOutputM5Speaker*)args;
      // The speaker has finished using this buffer; it can be refilled now.
      for (size_t i = 0; i < buf_count; ++i) { if (data == me->_buffer[i]) { me->_busy[i] = false; } }
    }
};


#define FFT_SIZE 256
class fft_t
{
  float _wr[FFT_SIZE + 1];
  float _wi[FFT_SIZE + 1];
  float _fr[FFT_SIZE + 1];
  float _fi[FFT_SIZE + 1];
  uint16_t _br[FFT_SIZE + 1];
  size_t _ie;

public:
  fft_t(void)
  {
#ifndef M_PI
#define M_PI 3.141592653
#endif
    _ie = logf( (float)FFT_SIZE ) / log(2.0) + 0.5;
    static constexpr float omega = 2.0f * M_PI / FFT_SIZE;
    static constexpr int s4 = FFT_SIZE / 4;
    static constexpr int s2 = FFT_SIZE / 2;
    for ( int i = 1 ; i < s4 ; ++i)
    {
    float f = cosf(omega * i);
      _wi[s4 + i] = f;
      _wi[s4 - i] = f;
      _wr[     i] = f;
      _wr[s2 - i] = -f;
    }
    _wi[s4] = _wr[0] = 1;

    size_t je = 1;
    _br[0] = 0;
    _br[1] = FFT_SIZE / 2;
    for ( size_t i = 0 ; i < _ie - 1 ; ++i )
    {
      _br[ je << 1 ] = _br[ je ] >> 1;
      je = je << 1;
      for ( size_t j = 1 ; j < je ; ++j )
      {
        _br[je + j] = _br[je] + _br[j];
      }
    }
  }

  void exec(const int16_t* in)
  {
    memset(_fi, 0, sizeof(_fi));
    for ( size_t j = 0 ; j < FFT_SIZE / 2 ; ++j )
    {
      float basej = 0.25 * (1.0-_wr[j]);
      size_t r = FFT_SIZE - j - 1;

      /// perform han window and stereo to mono convert.
      _fr[_br[j]] = basej * (in[j * 2] + in[j * 2 + 1]);
      _fr[_br[r]] = basej * (in[r * 2] + in[r * 2 + 1]);
    }

    size_t s = 1;
    size_t i = 0;
    do
    {
      size_t ke = s;
      s <<= 1;
      size_t je = FFT_SIZE / s;
      size_t j = 0;
      do
      {
        size_t k = 0;
        do
        {
          size_t l = s * j + k;
          size_t m = ke * (2 * j + 1) + k;
          size_t p = je * k;
          float Wxmr = _fr[m] * _wr[p] + _fi[m] * _wi[p];
          float Wxmi = _fi[m] * _wr[p] - _fr[m] * _wi[p];
          _fr[m] = _fr[l] - Wxmr;
          _fi[m] = _fi[l] - Wxmi;
          _fr[l] += Wxmr;
          _fi[l] += Wxmi;
        } while ( ++k < ke) ;
      } while ( ++j < je );
    } while ( ++i < _ie );
  }

  uint32_t get(size_t index)
  {
    return (index < FFT_SIZE / 2) ? (uint32_t)sqrtf(_fr[ index ] * _fr[ index ] + _fi[ index ] * _fi[ index ]) : 0u;
  }
};

// Stream buffer: about two seconds of a 128 kbps stream. Playback starts
// only once it is mostly full and pauses to refill when it runs low, so a
// network hiccup costs one gap instead of a burst of dropouts.
static constexpr const int preallocateBufferSize = 32 * 1024;
static constexpr const int bufferStartLevel = preallocateBufferSize * 3 / 4;
static constexpr const int bufferLowLevel = preallocateBufferSize / 8;
static constexpr const uint32_t bufferWaitMs = 3000;
static constexpr const int preallocateCodecSize = 29192; // MP3 codec max mem needed
static void* preallocateBuffer = nullptr;
static void* preallocateCodec = nullptr;
static constexpr size_t WAVE_SIZE = 320;
static AudioOutputM5Speaker out(&M5.Speaker, m5spk_virtual_channel);
static AudioGenerator *decoder = nullptr;
static AudioFileSourceICYStream *file = nullptr;
static AudioFileSourceBuffer *buff = nullptr;
static fft_t fft;
static bool fft_enabled = false;
static bool wave_enabled = false;
static uint16_t prev_y[(FFT_SIZE / 2)+1];
static uint16_t peak_y[(FFT_SIZE / 2)+1];
static int16_t wave_y[WAVE_SIZE];
static int16_t wave_h[WAVE_SIZE];
static int16_t raw_data[WAVE_SIZE * 2];
static int header_height = 0;
// Station numbers are shared between the UI and decode tasks, so use std::atomic.
static std::atomic<size_t> playing_index { 0 }; // station the decode task is on; the UI navigates relative to it
static char stream_title[128] = { 0 };
static const char* meta_text[2] = { nullptr, stream_title };
static const size_t meta_text_num = sizeof(meta_text) / sizeof(meta_text[0]);
static uint8_t meta_mod_bits = 0;
static std::atomic<size_t> playindex { ~0u }; // station requested from the UI; ~0u = none

static void MDCallback(void *cbData, const char *type, bool isUnicode, const char *string)
{
  (void)cbData;
  if ((strcmp(type, "StreamTitle") == 0) && (strcmp(stream_title, string) != 0))
  {
    strncpy(stream_title, string, sizeof(stream_title));
    meta_mod_bits |= 2;
  }
}

static void stop(void)
{
  if (decoder) {
    decoder->stop();
    delete decoder;
    decoder = nullptr;
  }

  if (buff) {
    buff->close();
    delete buff;
    buff = nullptr;
  }
  if (file) {
    file->close();
    delete file;
    file = nullptr;
  }
  out.stop();
}

/// Request a station. Requests are absolute station numbers, so the UI and
/// the decode task never update a shared counter.
static void play(size_t index)
{
  playindex = index % stations;
}

/// Ask for the station after `index` from the decode task - unless a
/// selection made from the UI is already waiting, which wins.
static void playNext(size_t index)
{
  if (++index >= stations) { index = 0; }
  size_t none = ~0u;
  playindex.compare_exchange_strong(none, index);
}

/// Sleep for `ms`, cut short when a station gets selected.
static void waitUnlessSelected(uint32_t ms)
{
  uint32_t start = millis();
  while (playindex == ~0u && millis() - start < ms) { M5.delay(10); }
}

/// Keep filling the stream buffer until it holds `level` bytes. Returns false
/// when a station was selected meanwhile; sets *timed_out when the stream
/// could not deliver within bufferWaitMs (play on with what there is).
static bool waitForBuffer(int level, bool* timed_out)
{
  *timed_out = false;
  uint32_t start = millis();
  while (buff->getFillLevel() < (uint32_t)level)
  {
    if (playindex != ~0u) { return false; }
    if (millis() - start >= bufferWaitMs) { *timed_out = true; break; }
    buff->loop();
    M5.delay(1);
  }
  return true;
}

/// A station is counted as working only once this much audio has actually
/// been played (buffering time does not count); one that fails earlier is
/// skipped like one that could not be opened.
static constexpr uint32_t stationOkSeconds = 2;

static void decodeTask(void*)
{
  size_t failures = 0;    // stations that failed in a row
  bool starved = false;   // the stream fell behind: play on, do not wait again until it caught up
  bool confirmed = false; // the current station has played for stationOkSeconds
  uint32_t frames_at_start = 0;
  for (;;)
  {
    M5.delay(1);
    if (playindex != ~0u)
    {
      auto index = playindex.exchange(~0u);
      if (index >= stations) { index = 0; }
      stop();
      playing_index = index;
      starved = false;
      confirmed = false;
      meta_text[0] = station_list[index][0];
      stream_title[0] = 0;
      meta_mod_bits = 3;
      file = new AudioFileSourceICYStream(station_list[index][1]);
      bool ok = file->isOpen();
      if (ok)
      {
        file->RegisterMetadataCB(MDCallback, (void*)"ICY");
        buff = new AudioFileSourceBuffer(file, preallocateBuffer, preallocateBufferSize);
        // The buffer's very first read fills it wholesale (replacing, not
        // adding to, anything loop() gathered before), so trigger that first,
        // then let loop() top it up before decoding starts.
        uint8_t dummy;
        buff->read(&dummy, 0);
        bool timed_out;
        if (!waitForBuffer(bufferStartLevel, &timed_out)) { continue; }
        starved = timed_out; // a slow stream: do not wait again until it catches up
        decoder = new AudioGeneratorMP3(preallocateCodec, preallocateCodecSize);
        ok = decoder->begin(buff, &out); // false if the stream broke meanwhile
      }
      if (ok)
      {
        frames_at_start = out.getFrames();
        continue;
      }
    }
    else if (decoder && decoder->isRunning())
    {
      if (!confirmed && out.getRate() != 0
       && out.getFrames() - frames_at_start >= out.getRate() * stationOkSeconds)
      {
        confirmed = true;
        failures = 0;
      }
      uint32_t level = buff->getFillLevel();
      if (starved)
      {
        if (level >= (uint32_t)bufferStartLevel) { starved = false; }
      }
      else if (level < (uint32_t)bufferLowLevel)
      { // running dry: refill before decoding on.
        bool timed_out;
        if (!waitForBuffer(bufferStartLevel, &timed_out)) { continue; }
        starved = timed_out;
      }
      if (decoder->loop()) { continue; }
      decoder->stop();
      if (confirmed)
      { // the stream ended or broke after playing: move on to the next station.
        playNext(playing_index);
        continue;
      }
    }
    else
    {
      continue;
    }
    // A station that could not be opened, broke during buffering, or stopped
    // before stationOkSeconds of audio came out: skip it. Once every station failed in a
    // row, wait a while before going round again. A selection from the UI
    // cuts the wait short.
    strncpy(stream_title, "(unavailable, skipping)", sizeof(stream_title) - 1);
    meta_mod_bits |= 2;
    stop();
    if (++failures >= stations)
    {
      failures = 0;
      waitUnlessSelected(5000);
    }
    else
    {
      waitUnlessSelected(500);
    }
    playNext(playing_index);
  }
}

static uint32_t bgcolor(LGFX_Device* gfx, int y)
{
  auto h = gfx->height();
  auto dh = h - header_height;
  int v = ((h - y)<<5) / dh;
  if (dh > 44)
  {
    int v2 = ((h - y - 1)<<5) / dh;
    if ((v >> 2) != (v2 >> 2))
    {
      return 0x666666u;
    }
  }
  return gfx->color888(v + 2, v, v + 6);
}

static void gfxSetup(LGFX_Device* gfx)
{
  if (gfx == nullptr) { return; }
  if (gfx->width() < gfx->height())
  {
    gfx->setRotation(gfx->getRotation()^1);
  }
  gfx->setFont(&fonts::lgfxJapanGothic_12);
  gfx->setEpdMode(epd_mode_t::epd_fastest);
  gfx->setTextWrap(false);
  gfx->setCursor(0, 8);
  gfx->println("WebRadio player");
  gfx->fillRect(0, 6, gfx->width(), 2, TFT_BLACK);

  header_height = (gfx->height() > 80) ? 33 : 21;
  fft_enabled = !gfx->isEPD();
  if (fft_enabled)
  {
    wave_enabled = (gfx->getBoard() != m5gfx::board_M5UnitLCD);

    for (int y = header_height; y < gfx->height(); ++y)
    {
      gfx->drawFastHLine(0, y, gfx->width(), bgcolor(gfx, y));
    }
  }

  for (int x = 0; x < (FFT_SIZE/2)+1; ++x)
  {
    prev_y[x] = INT16_MAX;
    peak_y[x] = INT16_MAX;
  }
  for (int x = 0; x < WAVE_SIZE; ++x)
  {
    wave_y[x] = gfx->height();
    wave_h[x] = 0;
  }
}

void gfxLoop(LGFX_Device* gfx)
{
  if (gfx == nullptr) { return; }
  if (header_height > 32)
  {
    if (meta_mod_bits)
    {
      gfx->startWrite();
      for (int id = 0; id < meta_text_num; ++id)
      {
        if (0 == (meta_mod_bits & (1<<id))) { continue; }
        meta_mod_bits &= ~(1<<id);
        size_t y = id * 12;
        if (y+12 >= header_height) { continue; }
        gfx->setCursor(4, 8 + y);
        gfx->fillRect(0, 8 + y, gfx->width(), 12, gfx->getBaseColor());
        gfx->print(meta_text[id]);
        gfx->print(" "); // Garbage data removal when UTF8 characters are broken in the middle.
      }
      gfx->display();
      gfx->endWrite();
    }
  }
  else
  {
    static int title_x;
    static int title_id;
    static int wait = INT16_MAX;

    if (meta_mod_bits)
    {
      if (meta_mod_bits & 1)
      {
        title_x = 4;
        title_id = 0;
        gfx->fillRect(0, 8, gfx->width(), 12, gfx->getBaseColor());
      }
      meta_mod_bits = 0;
      wait = 0;
    }

    if (--wait < 0)
    {
      int tx = title_x;
      int tid = title_id;
      wait = 3;
      gfx->startWrite();
      uint_fast8_t no_data_bits = 0;
      do
      {
        if (tx == 4) { wait = 255; }
        gfx->setCursor(tx, 8);
        const char* meta = meta_text[tid];
        if (meta[0] != 0)
        {
          gfx->print(meta);
          gfx->print("  /  ");
          tx = gfx->getCursorX();
          if (++tid == meta_text_num) { tid = 0; }
          if (tx <= 4)
          {
            title_x = tx;
            title_id = tid;
          }
        }
        else
        {
          if ((no_data_bits |= 1 << tid) == ((1 << meta_text_num) - 1))
          {
            break;
          }
          if (++tid == meta_text_num) { tid = 0; }
        }
      } while (tx < gfx->width());
      --title_x;
      gfx->display();
      gfx->endWrite();
    }
  }

  if (fft_enabled)
  {
    static int prev_x[2];
    static int peak_x[2];

    auto buf = out.getBuffer();
    if (buf)
    {
      memcpy(raw_data, buf, WAVE_SIZE * 2 * sizeof(int16_t)); // stereo data copy
      gfx->startWrite();

      // draw stereo level meter
      for (size_t i = 0; i < 2; ++i)
      {
        int32_t level = 0;
        for (size_t j = i; j < 640; j += 32)
        {
          uint32_t lv = abs(raw_data[j]);
          if (level < lv) { level = lv; }
        }

        int32_t x = (level * gfx->width()) / INT16_MAX;
        int32_t px = prev_x[i];
        if (px != x)
        {
          gfx->fillRect(x, i * 3, px - x, 2, px < x ? 0xFF9900u : 0x330000u);
          prev_x[i] = x;
        }
        px = peak_x[i];
        if (px > x)
        {
          gfx->writeFastVLine(px, i * 3, 2, TFT_BLACK);
          px--;
        }
        else
        {
          px = x;
        }
        if (peak_x[i] != px)
        {
          peak_x[i] = px;
          gfx->writeFastVLine(px, i * 3, 2, TFT_WHITE);
        }
      }
      gfx->display();

      // draw FFT level meter
      fft.exec(raw_data);
      size_t bw = gfx->width() / 60;
      if (bw < 3) { bw = 3; }
      int32_t dsp_height = gfx->height();
      int32_t fft_height = dsp_height - header_height - 1;
      size_t xe = gfx->width() / bw;
      if (xe > (FFT_SIZE/2)) { xe = (FFT_SIZE/2); }
      int32_t wave_next = ((header_height + dsp_height) >> 1) + (((256 - (raw_data[0] + raw_data[1])) * fft_height) >> 17);

      uint32_t bar_color[2] = { 0x000033u, 0x99AAFFu };

      for (size_t bx = 0; bx <= xe; ++bx)
      {
        size_t x = bx * bw;
        if ((x & 7) == 0) { gfx->display(); taskYIELD(); }
        int32_t f = fft.get(bx);
        int32_t y = (f * fft_height) >> 18;
        if (y > fft_height) { y = fft_height; }
        y = dsp_height - y;
        int32_t py = prev_y[bx];
        if (y != py)
        {
          gfx->fillRect(x, y, bw - 1, py - y, bar_color[(y < py)]);
          prev_y[bx] = y;
        }
        py = peak_y[bx] + 1;
        if (py < y)
        {
          gfx->writeFastHLine(x, py - 1, bw - 1, bgcolor(gfx, py - 1));
        }
        else
        {
          py = y - 1;
        }
        if (peak_y[bx] != py)
        {
          peak_y[bx] = py;
          gfx->writeFastHLine(x, py, bw - 1, TFT_WHITE);
        }


        if (wave_enabled)
        {
          for (size_t bi = 0; bi < bw; ++bi)
          {
            size_t i = x + bi;
            if (i >= gfx->width() || i >= WAVE_SIZE) { break; }
            y = wave_y[i];
            int32_t h = wave_h[i];
            bool use_bg = (bi+1 == bw);
            if (h>0)
            { /// erase previous wave.
              gfx->setAddrWindow(i, y, 1, h);
              h += y;
              do
              {
                uint32_t bg = (use_bg || y < peak_y[bx]) ? bgcolor(gfx, y)
                            : (y == peak_y[bx]) ? 0xFFFFFFu
                            : bar_color[(y >= prev_y[bx])];
                gfx->writeColor(bg, 1);
              } while (++y < h);
            }
            size_t i2 = i << 1;
            int32_t y1 = wave_next;
            wave_next = ((header_height + dsp_height) >> 1) + (((256 - (raw_data[i2] + raw_data[i2 + 1])) * fft_height) >> 17);
            int32_t y2 = wave_next;
            if (y1 > y2)
            {
              int32_t tmp = y1;
              y1 = y2;
              y2 = tmp;
            }
            y = y1;
            h = y2 + 1 - y;
            wave_y[i] = y;
            wave_h[i] = h;
            if (h>0)
            { /// draw new wave.
              gfx->setAddrWindow(i, y, 1, h);
              h += y;
              do
              {
                uint32_t bg = (y < prev_y[bx]) ? 0xFFCC33u : 0xFFFFFFu;
                gfx->writeColor(bg, 1);
              } while (++y < h);
            }
          }
        }
      }
      gfx->display();
      gfx->endWrite();
    }
  }

  if (!gfx->displayBusy())
  { // draw volume bar
    static int px;
    uint8_t v = M5.Speaker.getVolume();
    int x = v * (gfx->width()) >> 8;
    if (px != x)
    {
      gfx->fillRect(x, 6, px - x, 2, px < x ? 0xAAFFAAu : 0u);
      gfx->display();
      px = x;
    }
  }
}

void setup(void)
{
  auto cfg = M5.config();

  // If you want to play sound from ModuleDisplay, write this
//  cfg.external_speaker.module_display = true;

  // If you want to play sound from ModuleRCA, write this
//  cfg.external_speaker.module_rca     = true;

  // If you want to play sound from HAT Speaker, write this
  cfg.external_speaker.hat_spk        = true;

  // If you want to play sound from HAT Speaker2, write this
//  cfg.external_speaker.hat_spk2       = true;

  // If you want to play sound from ATOMIC Speaker, write this
  cfg.external_speaker.atomic_spk     = true;

  M5.begin(cfg);
  out.setup();

  preallocateBuffer = malloc(preallocateBufferSize);
  preallocateCodec = malloc(preallocateCodecSize);
  if (!preallocateBuffer || !preallocateCodec) {
    M5.Display.printf("FATAL ERROR:  Unable to preallocate %d bytes for app\n", preallocateBufferSize + preallocateCodecSize);
    for (;;) { M5.delay(1000); }
  }

  { /// custom setting
    auto spk_cfg = M5.Speaker.config();
    /// Increasing the sample_rate will improve the sound quality instead of increasing the CPU load.
    spk_cfg.sample_rate = 48000; // default:64000 (64kHz)  e.g. 48000 , 50000 , 80000 , 96000 , 100000 , 128000 , 144000 , 192000 , 200000
    spk_cfg.task_pinned_core = APP_CPU_NUM;
    M5.Speaker.config(spk_cfg);
  }


  M5.Speaker.begin();

  M5.Display.println("Connecting to WiFi");
  WiFi.disconnect();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  // Modem sleep makes the station receive only at DTIM intervals; a stream
  // then arrives in bursts and the buffer drains between them.
  WiFi.setSleep(false);

#if defined ( WIFI_SSID ) &&  defined ( WIFI_PASS )
  WiFi.begin(WIFI_SSID, WIFI_PASS);
#else
  WiFi.begin();
#endif

  // Try forever
  while (WiFi.status() != WL_CONNECTED) {
    M5.Display.print(".");
    M5.delay(100);
  }
  M5.Display.clear();

  gfxSetup(&M5.Display);

  play(0);

  xTaskCreatePinnedToCore(decodeTask, "decodeTask", 4096, nullptr, 1, nullptr, PRO_CPU_NUM);
}

void loop(void)
{
  gfxLoop(&M5.Display);

  {
    static int prev_frame;
    int frame;
    do
    {
      M5.delay(1);
    } while (prev_frame == (frame = millis() >> 3)); /// 8 msec cycle wait
    prev_frame = frame;
  }

  M5.update();
  auto td = M5.Touch.getDetail();
  if (M5.BtnA.wasPressed() || td.wasPressed())
  {
    M5.Speaker.tone(440, 50);
  }
  int cc = M5.BtnA.getClickCount();
  if (cc == 0) cc = td.getClickCount();
  if (M5.BtnA.wasDecideClickCount() || td.wasClicked())
  {
    switch (cc)
    {
    case 1:
      M5.Speaker.tone(1000, 100);
      play(playing_index + 1);
      break;

    case 2:
      M5.Speaker.tone(800, 100);
      play(playing_index + stations - 1);
      break;
    }
  }
  if (M5.BtnA.isHolding() || M5.BtnB.isPressed() || M5.BtnC.isPressed() || td.isHolding())
  {
    size_t v = M5.Speaker.getVolume();
    int add = (M5.BtnB.isPressed()) ? -1 : 1;
    if (M5.BtnA.isHolding() || td.isHolding())
    {
      add = (M5.BtnA.getClickCount() || td.getClickCount()) ? -1 : 1;
    }
    v += add;
    if (v <= 255)
    {
      M5.Speaker.setVolume(v);
    }
  }
}
