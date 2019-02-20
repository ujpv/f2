#include "particle.h"

#include <stdexcept>
#include <random>

ObjectWithLifeTime::ObjectWithLifeTime()
  : m_startTime_ms(0)
  , m_lifeDuration_ms(0)
{}

bool ObjectWithLifeTime::isExpired(int64_t time_ms) const {
  return time_ms - m_startTime_ms > m_lifeDuration_ms;
}

void ObjectWithLifeTime::setLifeTime(int64_t startTime_ms, int64_t lifeDuration_ms)
{
  m_startTime_ms = startTime_ms;
  m_lifeDuration_ms = lifeDuration_ms;
}

int64_t ObjectWithLifeTime::getLifeTimeDur() const {
  return m_lifeDuration_ms;
}

ObjectWithLifeTime::~ObjectWithLifeTime() {}

Vec2f &Vec2f::operator+=(const Vec2f &rhs)
{
  x += rhs.x;
  y += rhs.y;
  return *this;
}

Vec2f Vec2f::operator+(const Vec2f &rhs) const
{
  return Vec2f(x + rhs.x, y + rhs.y);
}

Vec2f Vec2f::rotated(float angle) const
{
  angle *= (3.14 / 180); // to radians
  float cs = std::cos(angle);
  float sn = std::sin(angle);
  return Vec2f(x * cs - y * sn, x * sn + y * cs);
}

Vec2f Vec2f::makeRandom(float minLength, float maxLength)
{
  if (minLength > maxLength)
    throw std::logic_error("minLength > maxLength");

  static std::default_random_engine re;
  std::uniform_real_distribution<float> lengthDist(minLength, maxLength);
  std::uniform_real_distribution<float> angeDist(0.0, 360.0);

  Vec2f result {Vec2f{lengthDist(re), 0.0}.rotated(angeDist(re))};
  return result;
}
