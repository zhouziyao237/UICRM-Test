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

#include "MotorCanBase.h"
#include "utils.h"

namespace control {

    Gimbal::Gimbal(gimbal_t gimbal) {
        pitch_motor_ = gimbal.pitch_motor;
        yaw_motor_ = gimbal.yaw_motor;
        data_ = gimbal.data;

        pitch_angle_ = 0;
        yaw_angle_ = 0;

        pitch_motor_->RegisterDataUpdateCallback(UpdataPitchByMotor, this);
        yaw_motor_->RegisterDataUpdateCallback(UpdataYawByMotor, this);

        pitch_lower_limit_ = -data_.pitch_max_;
        pitch_upper_limit_ = data_.pitch_max_;
        yaw_lower_limit_ = -data_.yaw_max_;
        yaw_upper_limit_ = data_.yaw_max_;
    }

    Gimbal::~Gimbal() {
    }

    gimbal_data_t* Gimbal::GetData() {
        return &data_;
    }

    void Gimbal::Update() {
        pitch_angle_ = clip<float>(pitch_angle_, pitch_lower_limit_, pitch_upper_limit_);
        pitch_motor_->SetTarget((pitch_angle_ + data_.pitch_offset_) * data_.pitch_reduction);

        if (!data_.yaw_circle_) {
            yaw_angle_ = clip<float>(yaw_angle_, yaw_lower_limit_, yaw_upper_limit_);
        }
        yaw_motor_->SetTarget((yaw_angle_ + data_.yaw_offset_) * data_.yaw_reduction);
    }

    void Gimbal::UpdateIMU(float pitch, float yaw) {
        float pt_diff = pitch_angle_ - pitch;
        float actual_pitch_angle = getPitchByMotor();
        float new_pitch_diff = clip<float>(pt_diff + actual_pitch_angle, pitch_lower_limit_, pitch_upper_limit_);
        new_pitch_diff = new_pitch_diff - actual_pitch_angle;
        if (pt_diff != new_pitch_diff) {
            pitch_angle_ = pitch_angle_ + new_pitch_diff - pt_diff;
        }
        pt_diff = wrap<float>(new_pitch_diff, -PI, PI);
























































































        if (abs(pt_diff) < data_.pitch_eposition) {
            pt_diff = 0;
        }
        pitch_motor_->SetTarget((pt_diff + actual_pitch_angle + data_.pitch_offset_) * data_.pitch_reduction);

        float yt_diff = yaw_angle_ - yaw;
        float actual_yaw_angle = getYawByMotor();
        if (!data_.yaw_circle_) {
            float new_yaw_diff = clip<float>(yt_diff + actual_yaw_angle, yaw_lower_limit_, yaw_upper_limit_);
            new_yaw_diff = new_yaw_diff - actual_yaw_angle;
            if (yt_diff != new_yaw_diff) {
                yaw_angle_ = yaw_angle_ + new_yaw_diff - yt_diff;
            }
            yt_diff = wrap<float>(new_yaw_diff, -PI, PI);
        } else {
            yt_diff = wrap<float>(yt_diff, -PI, PI);
        }

        if (abs(yt_diff) < data_.yaw_eposition) {
            yt_diff = 0;
        }

        yaw_motor_->SetTarget((yt_diff + actual_yaw_angle + data_.yaw_offset_) * data_.yaw_reduction);
    }

    void Gimbal::TargetAbs(float abs_pitch, float abs_yaw) {
        if (data_.pitch_inverted)
            abs_pitch = -abs_pitch;
        if (data_.yaw_inverted)
            abs_yaw = -abs_yaw;
        pitch_angle_ = clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);
        if (data_.yaw_circle_) {
            yaw_angle_ = wrap<float>(abs_yaw, -PI, PI);
        } else {
            yaw_angle_ = clip<float>(abs_yaw, -data_.yaw_max_, data_.yaw_max_);
        }
    }

    void Gimbal::TargetRel(float rel_pitch, float rel_yaw) {
        if (data_.pitch_inverted)
            rel_pitch = -rel_pitch;
        if (data_.yaw_inverted)
            rel_yaw = -rel_yaw;
        pitch_angle_ = pitch_angle_ + rel_pitch;
        yaw_angle_ = yaw_angle_ + rel_yaw;
    }

    void Gimbal::UpdateOffset(float pitch_offset, float yaw_offset) {
        data_.pitch_offset_ = wrap<float>(pitch_offset + data_.pitch_offset_, 0, 2 * PI);
        data_.yaw_offset_ = wrap<float>(yaw_offset + data_.yaw_offset_, 0, 2 * PI);
    }

    void Gimbal::TargetReal(float new_pitch, float new_yaw) {
        if (data_.pitch_inverted)
            new_pitch = -new_pitch;
        if (data_.yaw_inverted)
            new_yaw = -new_yaw;
        float current_pitch = getPitchByMotor();
        float current_yaw = getYawByMotor();
        pitch_angle_ = current_pitch + new_pitch;
        yaw_angle_ = current_yaw + new_yaw;
    }

    float Gimbal::getPitchTarget() const {
        return pitch_angle_;
    }

    float Gimbal::getYawTarget() const {
        return yaw_angle_;
    }

    void Gimbal::UpdataPitchByMotor(void* arg) {
        Gimbal* self = static_cast<Gimbal*>(arg);
        float motor_angle = self->pitch_motor_->GetTheta();
        
        float gimbal_angle = motor_angle / self->data_.pitch_reduction;
        self->pitch_wrap_.theta = wrap<float>(gimbal_angle - self->data_.pitch_offset_, -PI, PI);
    }

    void Gimbal::UpdataYawByMotor(void* arg){
        Gimbal* self = static_cast<Gimbal*>(arg);
        float motor_angle = self->yaw_motor_->GetTheta();

        if (self->data_.yaw_reduction != 1){
            if (!self->yaw_wrap_.initialized_) {
                self->yaw_wrap_.last_motor_angle_ = motor_angle;
                self->yaw_wrap_.initialized_ = true;
            } else {
                float angle_diff = motor_angle - self->yaw_wrap_.last_motor_angle_;

                if (angle_diff < -PI) {
                    self->yaw_wrap_.offset_angle_ += 2 * PI / self->data_.yaw_reduction;
                } else if (angle_diff > PI) {
                    self->yaw_wrap_.offset_angle_ -= 2 * PI / self->data_.yaw_reduction;
                }

                self->yaw_wrap_.last_motor_angle_ = motor_angle;
            }

            float gimbal_angle = wrap<float>(
                self->yaw_wrap_.offset_angle_ + motor_angle / self->data_.yaw_reduction,
                0, 2 * PI);
            self->yaw_wrap_.theta = wrap<float>(gimbal_angle - self->data_.yaw_offset_, -PI, PI);
        } else {
            self->yaw_wrap_.theta = self->yaw_motor_->GetTheta();
        }
    }

    float Gimbal::getPitchByMotor() {
        return pitch_wrap_.theta - data_.pitch_offset_;
    }

    float Gimbal::getYawByMotor() {
        return yaw_wrap_.theta - data_.yaw_offset_;
    }

    int Gimbal::GetDebugString(char* buffer, size_t size) const {
        float pitch_motor_raw = pitch_motor_->GetTheta();
        float yaw_motor_raw = yaw_motor_->GetTheta();

        float pitch_gimbal_actual = const_cast<Gimbal*>(this)->getPitchByMotor();
        float yaw_gimbal_actual = const_cast<Gimbal*>(this)->getYawByMotor();
        float pitch_gimbal_target = pitch_angle_;
        float yaw_gimbal_target = yaw_angle_;

        return snprintf(buffer, size,
            "P_Mtr:%2.4f|P_GimAct:%2.4f|P_GimTgt:%2.4f|PitchTheta:%2.4f|\r\n"
            "Y_Mtr:%2.4f|Y_GimAct:%2.4f|Y_GimTgt:%2.4f|YawTheta:%2.4f|\r\n",
            pitch_motor_raw, pitch_gimbal_actual, pitch_gimbal_target, pitch_wrap_.theta,
            yaw_motor_raw, yaw_gimbal_actual, yaw_gimbal_target, yaw_wrap_.theta);
    }

}  // namespace control