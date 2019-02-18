#include <math.h>
#include <algorithm>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <random>
#include <cassert>
#include <iostream>

//#include "./nvToolsExt.h"

#include "test.h"
#include "particle.h"

namespace  {

// Limits
constexpr size_t SPARKS_COUNT = 64;
constexpr size_t MAX_POINTS = 2048 * SPARKS_COUNT;
constexpr size_t MAX_EVENETS = 100;

// Effect consts
constexpr float MIN_START_SPEED = 0; // in (px / sec)
constexpr float MAX_START_SPEED = 100; // in (px / sec)
constexpr float GRAVIRY = -100.0; // in (px / sec ^ 2)
constexpr int64_t MIN_LIFE_TIME = 1; // in sec
constexpr int64_t MAX_LIFE_TIME = 2; // in sec
constexpr float BURST_PROBABILITY = 2 /64.f; // 0..1

// Defenitions
struct Color { float r, g, b, a; };

class Particle : public AbstractParticle<Vec2f> {
public:
  Particle(const Vec2f& position,
           const Vec2f& velocity,
           int64_t creationTime,
           int64_t lifeTime,
           const Color& color)
    : m_color(color)
  {
    setVelocity(velocity);
    setPosition(position);
    setLifeTime(creationTime, lifeTime);
  }

  void draw() const {
    platform::drawPoint(getPosition().x, getPosition().y,
                        m_color.r, m_color.g, m_color.b, m_color.a);
  }

private:
  Color m_color;
};

class SpinLock {
  std::atomic_flag locked;
public:
  void lock() { while (locked.test_and_set(std::memory_order::memory_order_acquire)); }
  void unlock() {locked.clear(std::memory_order::memory_order_release);}
};

// Firework states:
std::atomic<int64_t> globalTimeMs;
std::atomic<bool> exitFlag{false};

// state:
// 1st bit - active buffer number
// 2nd bit - buffer swapping not allowed
std::atomic<unsigned> state{0b0};

std::vector<Vec2f> clicks;
SpinLock clicksMutex;

using Buffer = std::vector<Particle>;
std::array<Buffer, 2> buffers;

// Utils:
float randomValue(float min, float max)
{
  assert(min <= max);
  static std::default_random_engine re;
  std::uniform_real_distribution<float> dist(min, max);
  return dist(re);
}

bool randomBinary(float probability)
{
  assert(probability >= 0.f && probability <= 1.f);
  return randomValue(0, 1.f) < probability;
}

Color randomColor()
{
  std::array<float, 3> colors;
  colors[0] = randomValue(0.f, 1.f);
  colors[1] = randomValue(0.f, 1.f - colors[0]);
  colors[2] = randomValue(0.f, 1.f - colors[0] - colors[1]);
  std::random_shuffle(colors.begin(), colors.end());
  return {colors[0], colors[1], colors[2], 1.f};
}

void makeBurst(Vec2f position, Buffer &out)
{
  if (out.size() + SPARKS_COUNT >= MAX_POINTS)
    return;

  const auto color = randomColor();
  int64_t timeNow = globalTimeMs.load();
  for (size_t i = 0; i < SPARKS_COUNT; ++i) {
    out.emplace_back(position,
                     Vec2f::makeRandom(MIN_START_SPEED, MAX_START_SPEED), // speed
                     timeNow,
                     randomValue(MIN_LIFE_TIME * 1000, MAX_LIFE_TIME * 1000), // lifetime ms
                     color);
  }
}

// Mechanic logic:
void WorkerThread(void)
{
  while (!exitFlag)
  {
    //nvtxRangePush(__FUNCTION__);

    static int64_t lastTime = 0;
    const auto time = globalTimeMs.load();
    const auto delta = time - lastTime;

    if (delta < 10)
      std::this_thread::sleep_for(
            std::chrono::milliseconds(10 - static_cast<int>(delta)));
    lastTime = time;

    unsigned activeBuffer = state.load() & 1;
    auto& srcBuffer = buffers[activeBuffer];
    auto& dstBuffer = buffers[1 - activeBuffer];

    dstBuffer.clear();

    for (const auto &spark: srcBuffer) {
      auto x = spark.getPosition().x;
      auto y = spark.getPosition().y;
      if (x < 0 || y < 0 ||
          x > test::SCREEN_WIDTH || y > test::SCREEN_HEIGHT)
      {
        continue;
      }

      if (spark.isExpired(time)) {
        if (randomBinary(BURST_PROBABILITY))
          makeBurst(spark.getPosition(), dstBuffer);

        continue;
      }

      if (dstBuffer.size() >= MAX_POINTS)
        break;

      dstBuffer.push_back(spark);
      dstBuffer.back().applayForce(float(delta), Vec2f{0, GRAVIRY});
      dstBuffer.back().update(delta);
    }

    if (not clicks.empty()) {
      std::lock_guard<SpinLock> guard{clicksMutex};
      for (auto& c: clicks)
        makeBurst(c, dstBuffer);
      clicks.clear();
    }

    // swap buffers:
    const unsigned newState = (1 - activeBuffer);
    bool swapped = false;
    while (not swapped) {
      unsigned currentState = activeBuffer;
      swapped = state.compare_exchange_weak(currentState, newState);
      // if not allowed give a chance to finish rendering
      if (not swapped)
        std::this_thread::yield();
    }
  } // while (!exitFlag)
} // void WorkerThread(void)

} // anonymous namespace

void test::init(void)
{
  // allocate memory for buffers and clicks events
  for (auto& b: buffers)
    b.reserve(MAX_POINTS);

  clicks.reserve(MAX_EVENETS);

  std::thread workerThread(WorkerThread);
  workerThread.detach(); // Glut + MSVC = join hangs in atexit()
}

void test::term(void)
{
  exitFlag = true;
}

void test::render(void)
{
  unsigned activeBuffer = 0;
  bool captured = false;

  while (not captured) {
    unsigned stateExpected = activeBuffer;
    unsigned newState = 2 | activeBuffer; // 2 = 0b10
    captured = state.compare_exchange_weak(stateExpected, newState);
    if (not captured)
      activeBuffer = stateExpected & 1;
  }

  for (size_t i = 0; i < buffers[activeBuffer].size(); ++i)
    buffers[activeBuffer][i].draw();

  state.exchange(activeBuffer);
}

void test::update(int dt)
{
  auto time = globalTimeMs.load();
  globalTimeMs.exchange(time + dt);
}

void test::on_click(int x, int y)
{
  y = int(SCREEN_HEIGHT) - y;
  std::lock_guard<SpinLock> lock(clicksMutex);
  clicks.emplace_back(x, y);
}
