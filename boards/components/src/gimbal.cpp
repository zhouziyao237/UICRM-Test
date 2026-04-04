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

#include "gimbal.h"

#include <cmath>

#include "MotorCanBase.h"
#include "bsp_error_handler.h"
#include "utils.h"

namespace control {

    namespace {
        constexpr float kTwoPi = 2.0f * PI;
        constexpr float kCoordinateEpsilon = 1.0e-5f;
    }

    float Gimbal::NearestEquivalentAngle(float configured_angle, float reference_angle) {
        // A stored offset may be written as -0.5 rad while the single-turn
        // encoder reports the same physical point as 2PI-0.5. Select the
        // equivalent branch nearest to the current continuous measurement.
        return configured_angle +
               roundf((reference_angle - configured_angle) / kTwoPi) * kTwoPi;
    }

    float Gimbal::PitchLogicalToMotor(float logical_delta) const {
        return data_.pitch_inverted ? -logical_delta : logical_delta;
    }

    float Gimbal::PitchMotorToLogical(float motor_delta) const {
        return data_.pitch_inverted ? -motor_delta : motor_delta;
    }

    void Gimbal::RefreshLinearCoordinates() {
        const float actual_pitch = pitch_motor_->GetOutputShaftTheta();
        const float new_pitch_center =
            NearestEquivalentAngle(data_.pitch_offset_, actual_pitch);

        if (!pitch_coordinate_initialized_) {
            pitch_angle_ += new_pitch_center - pitch_center_coordinate_;
            pitch_center_coordinate_ = new_pitch_center;
            pitch_coordinate_initialized_ = true;
        } else if (fabsf(new_pitch_center - pitch_center_coordinate_) > PI) {
            // The motor feedback crossed the 0/2PI encoder branch. Move the
            // center, target and limits together by exactly one revolution;
            // the physical command therefore remains continuous.
            const float branch_shift = new_pitch_center - pitch_center_coordinate_;
            pitch_center_coordinate_ = new_pitch_center;
            pitch_angle_ += branch_shift;
        }

        pitch_lower_limit_ = pitch_center_coordinate_ - data_.pitch_max_;
        pitch_upper_limit_ = pitch_center_coordinate_ + data_.pitch_max_;

        if (!data_.yaw_circle_) {
            const float actual_yaw = yaw_motor_->GetOutputShaftTheta();
            const float new_yaw_center =
                NearestEquivalentAngle(data_.yaw_offset_, actual_yaw);

            if (!yaw_coordinate_initialized_) {
                yaw_angle_ += new_yaw_center - yaw_center_coordinate_;
                yaw_center_coordinate_ = new_yaw_center;
                yaw_coordinate_initialized_ = true;
            } else if (fabsf(new_yaw_center - yaw_center_coordinate_) > PI) {
                const float branch_shift = new_yaw_center - yaw_center_coordinate_;
                yaw_center_coordinate_ = new_yaw_center;
                yaw_angle_ += branch_shift;
            }

            yaw_lower_limit_ = yaw_center_coordinate_ - data_.yaw_max_;
            yaw_upper_limit_ = yaw_center_coordinate_ + data_.yaw_max_;
        }
    }

    Gimbal::Gimbal(gimbal_t gimbal) {
        pitch_motor_ = gimbal.pitch_motor;
        yaw_motor_ = gimbal.yaw_motor;
        data_ = gimbal.data;

        RM_ASSERT_TRUE(pitch_motor_ != nullptr, "Pitch motor must not be null");
        RM_ASSERT_TRUE(yaw_motor_ != nullptr, "Yaw motor must not be null");
        RM_ASSERT_GE(data_.pitch_max_, 0.0f, "Pitch range must be non-negative");
        RM_ASSERT_GE(data_.yaw_max_, 0.0f, "Yaw range must be non-negative");

        pitch_center_coordinate_ = data_.pitch_offset_;
        pitch_angle_ = pitch_center_coordinate_;
        pitch_lower_limit_ = pitch_center_coordinate_ - data_.pitch_max_;
        pitch_upper_limit_ = pitch_center_coordinate_ + data_.pitch_max_;

        if (data_.yaw_circle_) {
            yaw_center_coordinate_ = wrap<float>(data_.yaw_offset_, 0.0f, kTwoPi);
            yaw_angle_ = yaw_center_coordinate_;
            yaw_lower_limit_ = 0.0f;
            yaw_upper_limit_ = kTwoPi;
            yaw_coordinate_initialized_ = true;
        } else {
            yaw_center_coordinate_ = data_.yaw_offset_;
            yaw_angle_ = yaw_center_coordinate_;
            yaw_lower_limit_ = yaw_center_coordinate_ - data_.yaw_max_;
            yaw_upper_limit_ = yaw_center_coordinate_ + data_.yaw_max_;
        }
    }

    Gimbal::~Gimbal() {
    }

    gimbal_data_t* Gimbal::GetData() {
        return &data_;
    }

    void Gimbal::Update() {
        RefreshLinearCoordinates();

        pitch_angle_ = clip<float>(pitch_angle_, pitch_lower_limit_, pitch_upper_limit_);
        pitch_motor_->SetTarget(pitch_angle_);

        if (data_.yaw_circle_) {
            yaw_angle_ = wrap<float>(yaw_angle_, 0.0f, kTwoPi);
        } else {
            yaw_angle_ = clip<float>(yaw_angle_, yaw_lower_limit_, yaw_upper_limit_);
        }
        yaw_motor_->SetTarget(yaw_angle_);
    }

    void Gimbal::UpdateIMU(float pitch, float yaw) {
        RefreshLinearCoordinates();

        // Pitch is a mechanically limited linear axis. All external values
        // use the logical gimbal direction; inversion is applied only when
        // converting the correction to the motor/output-shaft coordinate.
        const float desired_world_pitch =
            PitchMotorToLogical(pitch_angle_ - pitch_center_coordinate_);
        float logical_pitch_error = desired_world_pitch - pitch;
        if (fabsf(logical_pitch_error) < data_.pitch_eposition) {
            logical_pitch_error = 0.0f;
        }

        const float actual_pitch = pitch_motor_->GetOutputShaftTheta();
        const float motor_pitch_error =
            PitchLogicalToMotor(logical_pitch_error);
        const float unclipped_pitch_target = actual_pitch + motor_pitch_error;
        const float clipped_pitch_target =
            clip<float>(unclipped_pitch_target, pitch_lower_limit_, pitch_upper_limit_);
        const float applied_motor_error = clipped_pitch_target - actual_pitch;

        if (fabsf(applied_motor_error - motor_pitch_error) > kCoordinateEpsilon) {
            // Keep the stored world target consistent with the mechanical
            // correction that can actually be applied.
            const float applied_logical_error =
                PitchMotorToLogical(applied_motor_error);
            const float applied_world_pitch = pitch + applied_logical_error;
            pitch_angle_ = pitch_center_coordinate_ +
                           PitchLogicalToMotor(applied_world_pitch);
            pitch_angle_ =
                clip<float>(pitch_angle_, pitch_lower_limit_, pitch_upper_limit_);
        }
        pitch_motor_->SetTarget(clipped_pitch_target);

        const float actual_yaw = yaw_motor_->GetOutputShaftTheta();
        if (data_.yaw_circle_) {
            float yaw_error = wrap<float>(yaw_angle_ - data_.yaw_offset_ - yaw, -PI, PI);
            if (fabsf(yaw_error) < data_.yaw_eposition) {
                yaw_error = 0.0f;
            }
            yaw_motor_->SetTarget(actual_yaw + yaw_error);
        } else {
            const float desired_world_yaw = yaw_angle_ - yaw_center_coordinate_;
            float yaw_error = wrap<float>(desired_world_yaw - yaw, -PI, PI);
            if (fabsf(yaw_error) < data_.yaw_eposition) {
                yaw_error = 0.0f;
            }

            const float unclipped_yaw_target = actual_yaw + yaw_error;
            const float clipped_yaw_target =
                clip<float>(unclipped_yaw_target, yaw_lower_limit_, yaw_upper_limit_);
            const float applied_yaw_error = clipped_yaw_target - actual_yaw;

            if (fabsf(applied_yaw_error - yaw_error) > kCoordinateEpsilon) {
                yaw_angle_ = yaw_center_coordinate_ + yaw + applied_yaw_error;
                yaw_angle_ = clip<float>(yaw_angle_, yaw_lower_limit_, yaw_upper_limit_);
            }
            yaw_motor_->SetTarget(clipped_yaw_target);
        }
    }

    void Gimbal::TargetAbs(float abs_pitch, float abs_yaw) {
        RefreshLinearCoordinates();

        abs_pitch = PitchLogicalToMotor(abs_pitch);
        if (data_.yaw_inverted) {
            abs_yaw = -abs_yaw;
        }

        const float clipped_pitch =
            clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);
        pitch_angle_ = pitch_center_coordinate_ + clipped_pitch;

        const float clipped_yaw = clip<float>(abs_yaw, -data_.yaw_max_, data_.yaw_max_);
        if (data_.yaw_circle_) {
            yaw_angle_ = wrap<float>(clipped_yaw + data_.yaw_offset_, 0.0f, kTwoPi);
        } else {
            yaw_angle_ = yaw_center_coordinate_ + clipped_yaw;
        }
    }

    void Gimbal::TargetRel(float rel_pitch, float rel_yaw) {
        RefreshLinearCoordinates();

        rel_pitch = PitchLogicalToMotor(rel_pitch);
        if (data_.yaw_inverted) {
            rel_yaw = -rel_yaw;
        }

        pitch_angle_ =
            clip<float>(pitch_angle_ + rel_pitch, pitch_lower_limit_, pitch_upper_limit_);

        if (data_.yaw_circle_) {
            yaw_angle_ = wrap<float>(yaw_angle_ + rel_yaw, 0.0f, kTwoPi);
        } else {
            yaw_angle_ = clip<float>(yaw_angle_ + rel_yaw, yaw_lower_limit_, yaw_upper_limit_);
        }
    }

    void Gimbal::UpdateOffset(float pitch_offset, float yaw_offset) {
        RefreshLinearCoordinates();

        data_.pitch_offset_ += pitch_offset;
        pitch_center_coordinate_ += pitch_offset;
        pitch_angle_ += pitch_offset;
        pitch_lower_limit_ += pitch_offset;
        pitch_upper_limit_ += pitch_offset;

        if (data_.yaw_circle_) {
            data_.yaw_offset_ = wrap<float>(data_.yaw_offset_ + yaw_offset, 0.0f, kTwoPi);
            yaw_center_coordinate_ = data_.yaw_offset_;
            yaw_angle_ = wrap<float>(yaw_angle_ + yaw_offset, 0.0f, kTwoPi);
        } else {
            data_.yaw_offset_ += yaw_offset;
            yaw_center_coordinate_ += yaw_offset;
            yaw_angle_ += yaw_offset;
            yaw_lower_limit_ += yaw_offset;
            yaw_upper_limit_ += yaw_offset;
        }
    }

    void Gimbal::TargetReal(float new_pitch, float new_yaw) {
        RefreshLinearCoordinates();

        new_pitch = PitchLogicalToMotor(new_pitch);
        if (data_.yaw_inverted) {
            new_yaw = -new_yaw;
        }

        pitch_angle_ = clip<float>(pitch_motor_->GetOutputShaftTheta() + new_pitch,
                                   pitch_lower_limit_, pitch_upper_limit_);

        if (data_.yaw_circle_) {
            yaw_angle_ = wrap<float>(yaw_motor_->GetOutputShaftTheta() + new_yaw, 0.0f,
                                     kTwoPi);
        } else {
            yaw_angle_ = clip<float>(yaw_motor_->GetOutputShaftTheta() + new_yaw,
                                     yaw_lower_limit_, yaw_upper_limit_);
        }
    }

    float Gimbal::getPitchTarget() const {
        return pitch_angle_;
    }

    float Gimbal::getYawTarget() const {
        return yaw_angle_;
    }

    float Gimbal::getPitchTargetLogical() const {
        return PitchMotorToLogical(pitch_angle_ - pitch_center_coordinate_);
    }

    float Gimbal::getPitchByMotor() const {
        const float actual = pitch_motor_->GetOutputShaftTheta();
        const float center = NearestEquivalentAngle(data_.pitch_offset_, actual);
        return PitchMotorToLogical(actual - center);
    }

    float Gimbal::getYawByMotor() const {
        const float actual = yaw_motor_->GetOutputShaftTheta();
        if (data_.yaw_circle_) {
            return wrap<float>(actual - data_.yaw_offset_, -PI, PI);
        }
        const float center = NearestEquivalentAngle(data_.yaw_offset_, actual);
        return actual - center;
    }

}  // namespace control
