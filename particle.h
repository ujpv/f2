#pragma once

#include <cmath>
#include <stdint.h>

template <typename VectorType>
class MechanicalObject {
public:
  void setPosition(const VectorType &position)
  {
    m_position = position;
  }

  void setVelocity(const VectorType &velocity)
  {
    m_velocity = velocity;
  }

  void applayForce(float deltaTimeMs,
                   VectorType acceleration)
  {
    m_velocity += (acceleration * (deltaTimeMs / 1000.f));
  }

  void update(float deltaTimeMs)
  {
    m_position += (m_velocity * (deltaTimeMs / 1000.));
  }

  VectorType getPosition() const
  {
    return m_position;
  }

  VectorType getVelocity() const
  {
    return m_velocity;
  }

  virtual ~MechanicalObject() {}

protected:
  VectorType m_position;
  VectorType m_velocity;
};

class ObjectWithLifeTime {
public:
  ObjectWithLifeTime();
  bool isExpired(int64_t time_ms) const;
  void setLifeTime(int64_t startTime_ms, int64_t lifeDuration_ms);
  int64_t getLifeTimeDur() const;
  virtual ~ObjectWithLifeTime();

private:
  int64_t m_startTime_ms;
  int64_t m_lifeDuration_ms;
};

template<typename VectorType>
class AbstractParticle
    : public MechanicalObject<VectorType>
    , public ObjectWithLifeTime
{};

struct Vec2f {
  Vec2f() : x (0), y(0) {}
  Vec2f(float x, float y): x(x), y(y) {}

  Vec2f operator+(const Vec2f& rhs) const;

  Vec2f &operator+=(const Vec2f& rhs);

  template<class U>
  Vec2f operator*(U rhs)
  {
    return Vec2f(x * rhs, y * rhs);
  }

  template<class U>
  Vec2f &operator*=(U rhs)
  {
    x *= rhs;
    y *= rhs;
    return *this;
  }

  Vec2f rotated(float angle) const;

  static Vec2f makeRandom(float minLength, float maxLength);

  float x, y;
};
