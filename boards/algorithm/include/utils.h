/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#pragma once

#include <cmath>
#include <type_traits>

#include "main.h"

/**
 * @brief 将输入值从一个数值范围映射到另一个数值范围
 *
 * 该函数使用线性插值算法将给定的输入值从原始范围(in_min~in_max)转换到目标范围(out_min~out_max)
 *
 * @param value 需要进行范围映射的输入值
 * @param in_min 输入值的最小范围边界
 * @param in_max 输入值的最大范围边界
 * @param out_min 输出值的最小范围边界
 * @param out_max 输出值的最大范围边界
 * @return 返回经过范围映射后的值，类型为模板参数T指定的类型
 */
template <typename T>
T map(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/**
 * @brief 将一个值限制在一个范围内
 *
 * @tparam T    值的类型
 * @param value 要限制的值
 * @param min   范围最小值
 * @param max   范围最大值
 *
 * @return 限制在范围内的值
 *
 * @note 如果min > max，行为未定义
 */
/**
 * @brief clip a value to fall into a given range
 *
 * @tparam T    type of the value
 * @param value value to the clipped
 * @param min   range min
 * @param max   range max
 *
 * @return clipped value that falls in the range [min, max]
 *
 * @note undefined behavior if min > max
 */
template <typename T>
T clip(T value, T min, T max) {
    return value < min ? min : (value > max ? max : value);
}

/**
 * @brief Apply a symmetric zero deadband around the origin.
 *
 * Values whose magnitude is less than or equal to threshold become zero.
 * Values outside the deadband are returned unchanged.
 *
 * @note T must be a signed numeric type.
 */
template <typename T>
T apply_deadband(T value, T threshold) {
    if (threshold < static_cast<T>(0)) threshold = -threshold;
    return (value >= -threshold && value <= threshold)
               ? static_cast<T>(0)
               : value;
}

/**
 * @brief Normalize a signed symmetric input to [-1, 1] and apply deadband.
 *
 * This is intended for joystick, dial, mouse-axis and similar inputs.
 */
template <typename T>
float normalize_symmetric(T value, T full_scale, float deadband = 0.0f) {
    if (full_scale <= static_cast<T>(0)) return 0.0f;
    const float normalized =
        clip(static_cast<float>(value) / static_cast<float>(full_scale),
             -1.0f, 1.0f);
    return apply_deadband(normalized, deadband);
}

/**
 * @brief Slew a two-dimensional vector toward a target without independently
 *        clipping the X/Y components and distorting the vector direction.
 *
 * @param target_x target X component
 * @param target_y target Y component
 * @param max_step maximum vector delta per call
 * @param current_x current/output X component, updated in place
 * @param current_y current/output Y component, updated in place
 */
inline void slew_vector_2d(float target_x, float target_y, float max_step,
                           float& current_x, float& current_y) {
    if (!(max_step > 0.0f)) {
        current_x = target_x;
        current_y = target_y;
        return;
    }

    const float delta_x = target_x - current_x;
    const float delta_y = target_y - current_y;
    const float delta_norm = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    if (!(delta_norm > max_step)) {
        current_x = target_x;
        current_y = target_y;
        return;
    }

    const float scale = max_step / delta_norm;
    current_x += delta_x * scale;
    current_y += delta_y * scale;
}

/**
 * @brief Wrap a value into one period of [min, max].
 *
 * Unlike the old one-step implementation, this function accepts values an
 * arbitrary number of periods away from the requested interval.  This is
 * required by ABSOLUTE motor axes whose internal encoder accumulator may keep
 * increasing for hundreds of turns.
 *
 * Values already inside [min, max] are preserved, including an exact max
 * endpoint.  Values outside the interval are returned in [min, max).
 */
template <typename T>
T wrap(T value, T min, T max) {
    static_assert(std::is_arithmetic<T>::value,
                  "wrap requires an arithmetic type");

    const T range = max - min;
    if (!(range > static_cast<T>(0))) return min;
    if (value >= min && value <= max) return value;

    if constexpr (std::is_floating_point<T>::value) {
        if (!std::isfinite(value)) return min;
        T wrapped = std::fmod(value - min, range);
        if (wrapped < static_cast<T>(0)) wrapped += range;
        return min + wrapped;
    } else {
        T wrapped = (value - min) % range;
        if (wrapped < static_cast<T>(0)) wrapped += range;
        return min + wrapped;
    }
}

/**
 * @brief 环绕一个值使其落入给定范围，此函数会不断环绕直到落入范围
 *
 * @tparam T    值的类型
 * @param value 要环绕的值
 * @param min   范围最小值
 * @param max   范围最大值
 *
 * @return 落入范围[min, max]的环绕值
 *
 */
/**
 * @brief Compatibility alias for the robust multi-period wrap helper.
 *
 * @tparam T    type of the value
 * @param value value to be wrapped around
 * @param min   range min
 * @param max   range max
 *
 * @return wrapped value in one period of [min, max]
 */
template <typename T>
T wrapc(T value, T min, T max) {
    return wrap<T>(value, min, max);
}

/**
 * @brief 将一个值限制范围后，环绕使其落入给定的环绕范围
 *
 * @tparam T        值的类型
 * @param value     要环绕的值
 * @param min       限制范围最小值
 * @param max       限制范围最大值
 * @param range_min 环绕范围最小值
 * @param range_max 环绕范围最大值
 *
 * @return 落入环绕范围[range_min, range_max]的环绕值
 *
 * @note 如果min > max，行为未定义
 */
/**
 * @brief clip a value to fall into a given range; can wrap around domain
 *
 * @tparam T        type of the value
 * @param value     value to the clipped
 * @param min       clipping range min
 * @param max       clipping range max
 * @param range_min domain range min
 * @param range_max domain range max
 *
 * @return clipped value that falls in the range [min, max]
 *
 * @note undefined behavior if min > max
 */
template <typename T>
T wrapping_clip(T value, T min, T max, T range_min, T range_max) {
    value = wrap<T>(value, range_min, range_max);
    if (max >= min) {
        return value < min ? min : (value > max ? max : value);
    } else {
        // min > max; wrap around
        T middle = (min + max) / 2;
        if (value <= max && value >= range_min) {
            return value;
        } else if (value >= min && value <= range_max) {
            return value;
        } else if (value > max && value < middle) {
            return max;
        } else {
            return min;
        }
    }
}

/**
 * @brief 取最大值
 *
 * @tparam T     值的类型
 * @param value1 第一个值
 * @param value2 第二个值
 *
 * @return 两个值中的最大值
 */
/**
 * @brief max of two values
 *
 * @tparam T     type of the value
 * @param value1 first value
 * @param value2 second value
 *
 * @return the max of the values
 */
template <typename T>
T max(T value1, T value2) {
    return value1 < value2 ? value2 : value1;
}

/**
 * @brief 取最小值
 *
 * @tparam T     值的类型
 * @param value1 第一个值
 * @param value2 第二个值
 *
 * @return 两个值中的最小值
 */
/**
 * @brief min of two values
 *
 * @tparam T     type of the value
 * @param value1 first value
 * @param value2 second value
 *
 * @return the min of the values
 */
template <typename T>
T min(T value1, T value2) {
    return value1 < value2 ? value1 : value2;
}

/**
 * @brief 零点判断
 *
 * @tparam T    值的类型
 * @param value 要判断的值
 * @param zero  零点
 *
 * @return -1 小于零点，1 大于零点，0 等于零点
 */
/**
 * @brief get sign of a value
 *
 * @tparam T    type of the value
 * @param value value to be tested
 * @param zero  zero point of that value
 *
 * @return -1 for less than, 1 for greater than, and 0 for equal to
 */
template <typename T>
int sign(T value, T zero) {
    return value < zero ? -1 : (value > zero ? 1 : 0);
}

/**
 * @brief 边缘判断
 * @tparam T 受判断的数据类型
 */
template <typename T>
class EdgeDetector {
  public:
    /**
     * @brief 构造函数
     * @param initial 初始值
     */
    /**
     * @brief constructor
     * @param initial initial value
     */
    explicit EdgeDetector(T initial);
    /**
     * @brief 输入信号
     * @param signal 信号
     */
    /**
     * @brief input signal
     * @param signal signal
     */
    void input(T signal);
    /**
     * @brief 判断值是否变化（包括positive和negative）
     * @return true 变化，false 未变化
     */
    /**
     * @brief check if the value has changed (including positive and negative)
     * @return true if changed, false otherwise
     */
    bool edge();
    /**
     * @brief 判断值是否变大
     * @return true 变大，false 未变大
     */
    /**
     * @brief check if the value has increased
     * @return true if increased, false otherwise
     */
    bool posEdge();
    /**
     * @brief 判断值是否变小
     * @return true 变小，false 未变小
     */
    /**
     * @brief check if the value has decreased
     * @return true if decreased, false otherwise
     */
    bool negEdge();

  private:
    T prev_;
    bool posEdge_;
    bool negEdge_;
};

/**
 * @brief 布尔边缘判断
 * @details 用于判断布尔值的变化
 */
/**
 * @brief boolean edge detector
 * @details used to detect changes in boolean values
 */
class BoolEdgeDetector {
  public:
    /**
     * @brief 构造函数
     * @param initial 初始值
     */
    /**
     * @brief constructor
     * @param initial initial value
     */
    BoolEdgeDetector(bool initial);
    BoolEdgeDetector();
    /** Reset the stored state without producing an edge. */
    void reset(bool initial = false);
    /**
     * @brief 输入信号
     * @param signal 信号
     */
    /**
     * @brief input signal
     * @param signal signal
     */
    void input(bool signal);
    /**
     * @brief 判断值是否变化（不论变高还是变低）
     * @return true 变化，false 未变化
     */
    /**
     * @brief check if the value has changed (including positive and negative)
     * @return true if changed, false otherwise
     */
    bool edge();
    /**
     * @brief 判断值是否变大
     * @return true 变大，false 未变大
     */
    /**
     * @brief check if the value has increased
     * @return true if increased, false otherwise
     */
    bool posEdge();
    /**
     * @brief 判断值是否变小
     * @return true 变小，false 未变小
     */
    /**
     * @brief check if the value has decreased
     * @return true if decreased, false otherwise
     */
    bool negEdge();
    /**
     * @brief 获取当前值
     * @return 当前值
     */
    /**
     * @brief get current value
     * @return current value
     */
    bool get();

  private:
    bool prev_;
    bool posEdge_;
    bool negEdge_;
};

/**
 * @brief 浮点数边缘判断
 * @details 用于判断浮点数的变化
 * @note 由于浮点数的精度问题，可能会出现误判
 */
/**
 * @brief float edge detector
 * @details used to detect changes in float values
 * @note due to the precision of float, there might be false positives
 */
class FloatEdgeDetector {
  public:
    /**
     * @brief 构造函数
     * @param initial   初始值
     * @param threshold 阈值
     */
    /**
     * @brief constructor
     * @param initial  initial value
     * @param threshold threshold
     */
    FloatEdgeDetector(float initial, float threshold = 0);
    /**
     * @brief reset the detector state without producing a synthetic edge
     *
     * @param initial new previous value
     */
    void reset(float initial);
    /**
     * @brief 输入数据
     * @param signal 输入的数据
     */
    /**
     * @brief input signal
     * @param signal signal
     */
    void input(float signal);
    /**
     * @brief 判断值是否变化（不论变高还是变低）
     * @return true 变化，false 未变化
     */
    /**
     * @brief check if the value has changed (including positive and negative)
     * @return true if changed, false otherwise
     */
    bool edge();
    /**
     * @brief 判断值是否变大
     * @return true 变大，false 未变大
     */
    /**
     * @brief check if the value has increased
     * @return true if increased, false otherwise
     */
    bool posEdge();
    /**
     * @brief 判断值是否变小
     * @return true 变小，false 未变小
     */
    /**
     * @brief check if the value has decreased
     * @return true if decreased, false otherwise
     */
    bool negEdge();

  private:
    float prev_;
    float threshold_;
    bool posEdge_;
    bool negEdge_;
};

/**
 * @brief 斜坡信号源
 * @details 用于产生斜坡信号，避免信号突变
 */
/**
 * @brief ramp signal source
 * @details used to generate ramp signal to avoid sudden changes
 */
class RampSource {
  public:
    /**
     * @brief 构造函数
     * @param initial 初始值
     * @param min     最小值
     * @param max     最大值
     * @param step    步长
     */
    /**
     * @brief constructor
     * @param initial initial value
     * @param min    min value
     * @param max   max value
     * @param step step
     */
    RampSource(float initial, float min, float max, float step);
    /**
     * @brief 计算下一个周期
     * @param input 输入值
     * @return 下一个周期的值
     */
    /**
     * @brief calculate next value
     * @param input input value
     * @return next value
     */
    float Calc(float input);
    /**
     * @brief 获取当前值
     * @return 当前值
     */
    /**
     * @brief get current value
     * @return current value
     */
    float Get();
    /**
     * @brief 获取最大值
     * @return 最大值
     */
    /**
     * @brief get max value
     * @return max value
     */
    float GetMax();
    /**
     * @brief 获取最小值
     * @return 最小值
     */
    /**
     * @brief get min value
     * @return min value
     */
    float GetMin();
    /**
     * @brief 设置最大值
     * @param max 最大值
     */
    /**
     * @brief set max value
     * @param max max value
     */
    void SetMax(float max);
    /**
     * @brief 设置最小值
     * @param min 最小值
     */
    /**
     * @brief set min value
     * @param min min value
     */
    void SetMin(float min);
    /**
     * @brief 强制覆写当前值
     * @param current 当前值
     */
    /**
     * @brief set current value
     * @param current current value
     */
    void SetCurrent(float current);

  private:
    float input_;
    float output_;
    float min_;
    float max_;
    float step_;
};

void EndianSwap(void* data, size_t size);

class Ease {
  public:
    Ease(float initial, float step);
    void SetTarget(float target);
    void Reset(float value);
    float Calc(float target);
    float Calc();
    float GetOutput();
    float GetTarget();
    bool IsAtTarget();

  private:
    float current_;
    float target_;
    float step_;
};

template <typename T>
inline T linear_interpolation(T src_min, T src_max, T dst_min, T dst_max, T value) {
    value = clip(value, src_min, src_max);
    return dst_min + (dst_max - dst_min) * (value - src_min) / (src_max - src_min);
}