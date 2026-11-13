#ifndef VECTOR2_H
#define VECTOR2_H

#include <cmath>
#include <concepts>

template <typename T>
concept Numeric = std::integral<T> || std::floating_point<T>;

template <Numeric T>
class vector2 {
public:
    T x, y;

    vector2() : x(0), y(0) {}

    vector2(T x_, T y_) {
        x = x_;
        y = y_;
    }

    template <Numeric To>
    constexpr vector2<To> convert() const {
        return vector2<To>(static_cast<To>(this->x), static_cast<To>(this->y));
    }

    vector2<T>& operator+=(const vector2<T>& rhs) {
        this->x += rhs.x;
        this->y += rhs.y;
        return *this;
    }

    vector2<T>& operator-=(const vector2<T>& rhs) {
        this->x -= rhs.x;
        this->y -= rhs.y;
        return *this;
    }

    vector2<T>& operator*=(float rhs) {
        this->x *= rhs;
        this->y *= rhs;
        return *this;
    }

    vector2<T>& operator/=(float rhs) {
        this->x /= rhs;
        this->y /= rhs;
        return *this;
    }

    bool operator==(const vector2<T>& rhs) const {
        return this->x == rhs.x && this->y == rhs.y;
    }

    bool operator!=(const vector2<T>& rhs) const {
        return !(*this == rhs);
    }

    friend vector2<T> operator+(vector2<T> lhs, const vector2<T>& rhs) {
        lhs += rhs;
        return lhs;
    }

    friend vector2<T> operator-(vector2<T> lhs, const vector2<T>& rhs) {
        lhs -= rhs;
        return lhs;
    }

    float distance_to(const vector2<T>& other) const {
        return std::sqrt(std::pow(other.x - this->x, 2) + std::pow(other.y - this->y, 2));
    }

    float magnitude() const {
        return std::sqrt(x * x + y * y);
    }

    vector2<T> normalized() const {
        float mag = this->magnitude();
        return vector2<float>(this->x / mag, this->y / mag);
    }
};

#endif
