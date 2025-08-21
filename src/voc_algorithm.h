/*
 * Copyright (c) 2022, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file voc_algorithm.h
 * @brief Sensirion의 VOC/NOx 가스 지수 알고리즘 헤더 파일
 *
 * SGP4x 센서에서 나오는 원시(raw) 신호를 기반으로 VOC(휘발성 유기 화합물) 및
 * NOx(질소 산화물) 지수를 계산하는 데 사용되는 알고리즘의 함수와 데이터 구조를 정의합니다.
 * 이 알고리즘은 주변 환경의 가스 농도 변화에 적응하며, 장기적인 안정성을 위해
 * 평균 및 분산 추정, 시그모이드 함수, 적응형 로우패스 필터 등의 기법을 사용합니다.
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GASINDEXALGORITHM_H_
#define GASINDEXALGORITHM_H_

#include <stdint.h>

#ifndef __cplusplus

#if __STDC_VERSION__ >= 199901L
#include <stdbool.h>
#else

#ifndef bool
#define bool int
#define true 1
#define false 0
#endif  // bool

#endif  // __STDC_VERSION__

#endif  // __cplusplus

// Should be set by the building toolchain
#ifndef LIBRARY_VERSION_NAME
#define LIBRARY_VERSION_NAME "3.2.0"
#endif

// 알고리즘 타입 상수
#define GasIndexAlgorithm_ALGORITHM_TYPE_VOC (0) ///< VOC 알고리즘
#define GasIndexAlgorithm_ALGORITHM_TYPE_NOX (1) ///< NOx 알고리즘

// 알고리즘의 각종 파라미터 및 상수 정의
#define GasIndexAlgorithm_DEFAULT_SAMPLING_INTERVAL (1.f)
#define GasIndexAlgorithm_INITIAL_BLACKOUT (45.f)
#define GasIndexAlgorithm_INDEX_GAIN (230.f)
#define GasIndexAlgorithm_SRAW_STD_INITIAL (50.f)
#define GasIndexAlgorithm_SRAW_STD_BONUS_VOC (220.f)
#define GasIndexAlgorithm_SRAW_STD_NOX (2000.f)
#define GasIndexAlgorithm_TAU_MEAN_HOURS (12.f)
#define GasIndexAlgorithm_TAU_VARIANCE_HOURS (12.f)
#define GasIndexAlgorithm_TAU_INITIAL_MEAN_VOC (20.f)
#define GasIndexAlgorithm_TAU_INITIAL_MEAN_NOX (1200.f)
#define GasIndexAlgorithm_INIT_DURATION_MEAN_VOC ((3600.f * 0.75f))
#define GasIndexAlgorithm_INIT_DURATION_MEAN_NOX ((3600.f * 4.75f))
#define GasIndexAlgorithm_INIT_TRANSITION_MEAN (0.01f)
#define GasIndexAlgorithm_TAU_INITIAL_VARIANCE (2500.f)
#define GasIndexAlgorithm_INIT_DURATION_VARIANCE_VOC ((3600.f * 1.45f))
#define GasIndexAlgorithm_INIT_DURATION_VARIANCE_NOX ((3600.f * 5.70f))
#define GasIndexAlgorithm_INIT_TRANSITION_VARIANCE (0.01f)
#define GasIndexAlgorithm_GATING_THRESHOLD_VOC (340.f)
#define GasIndexAlgorithm_GATING_THRESHOLD_NOX (30.f)
#define GasIndexAlgorithm_GATING_THRESHOLD_INITIAL (510.f)
#define GasIndexAlgorithm_GATING_THRESHOLD_TRANSITION (0.09f)
#define GasIndexAlgorithm_GATING_VOC_MAX_DURATION_MINUTES ((60.f * 3.f))
#define GasIndexAlgorithm_GATING_NOX_MAX_DURATION_MINUTES ((60.f * 12.f))
#define GasIndexAlgorithm_GATING_MAX_RATIO (0.3f)
#define GasIndexAlgorithm_SIGMOID_L (500.f)
#define GasIndexAlgorithm_SIGMOID_K_VOC (-0.0065f)
#define GasIndexAlgorithm_SIGMOID_X0_VOC (213.f)
#define GasIndexAlgorithm_SIGMOID_K_NOX (-0.0101f)
#define GasIndexAlgorithm_SIGMOID_X0_NOX (614.f)
#define GasIndexAlgorithm_VOC_INDEX_OFFSET_DEFAULT (100.f)
#define GasIndexAlgorithm_NOX_INDEX_OFFSET_DEFAULT (1.f)
#define GasIndexAlgorithm_LP_TAU_FAST (20.0f)
#define GasIndexAlgorithm_LP_TAU_SLOW (500.0f)
#define GasIndexAlgorithm_LP_ALPHA (-0.2f)
#define GasIndexAlgorithm_VOC_SRAW_MINIMUM (20000)
#define GasIndexAlgorithm_NOX_SRAW_MINIMUM (10000)
#define GasIndexAlgorithm_PERSISTENCE_UPTIME_GAMMA ((3.f * 3600.f))
#define GasIndexAlgorithm_TUNING_INDEX_OFFSET_MIN (1)
#define GasIndexAlgorithm_TUNING_INDEX_OFFSET_MAX (250)
#define GasIndexAlgorithm_TUNING_LEARNING_TIME_OFFSET_HOURS_MIN (1)
#define GasIndexAlgorithm_TUNING_LEARNING_TIME_OFFSET_HOURS_MAX (1000)
#define GasIndexAlgorithm_TUNING_LEARNING_TIME_GAIN_HOURS_MIN (1)
#define GasIndexAlgorithm_TUNING_LEARNING_TIME_GAIN_HOURS_MAX (1000)
#define GasIndexAlgorithm_TUNING_GATING_MAX_DURATION_MINUTES_MIN (0)
#define GasIndexAlgorithm_TUNING_GATING_MAX_DURATION_MINUTES_MAX (3000)
#define GasIndexAlgorithm_TUNING_STD_INITIAL_MIN (10)
#define GasIndexAlgorithm_TUNING_STD_INITIAL_MAX (5000)
#define GasIndexAlgorithm_TUNING_GAIN_FACTOR_MIN (1)
#define GasIndexAlgorithm_TUNING_GAIN_FACTOR_MAX (1000)
#define GasIndexAlgorithm_MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING (64.f)
#define GasIndexAlgorithm_MEAN_VARIANCE_ESTIMATOR__ADDITIONAL_GAMMA_MEAN_SCALING \
    (8.f)
#define GasIndexAlgorithm_MEAN_VARIANCE_ESTIMATOR__FIX16_MAX (32767.f)

/**
 * @struct GasIndexAlgorithmParams
 * @brief 가스 지수 알고리즘의 모든 파라미터와 상태를 저장하는 구조체
 *
 * 이 구조체는 알고리즘의 현재 상태와 설정된 튜닝 파라미터들을 포함합니다.
 * 사용자는 이 구조체의 인스턴스를 생성하고, 알고리즘 함수들에 포인터로 전달해야 합니다.
 */
typedef struct {
    int mAlgorithm_Type;
    float mSamplingInterval;
    float mIndex_Offset;
    int32_t mSraw_Minimum;
    float mGating_Max_Duration_Minutes;
    float mInit_Duration_Mean;
    float mInit_Duration_Variance;
    float mGating_Threshold;
    float mIndex_Gain;
    float mTau_Mean_Hours;
    float mTau_Variance_Hours;
    float mSraw_Std_Initial;
    float mUptime;
    float mSraw;
    float mGas_Index;
    bool m_Mean_Variance_Estimator___Initialized;
    float m_Mean_Variance_Estimator___Mean;
    float m_Mean_Variance_Estimator___Sraw_Offset;
    float m_Mean_Variance_Estimator___Std;
    float m_Mean_Variance_Estimator___Gamma_Mean;
    float m_Mean_Variance_Estimator___Gamma_Variance;
    float m_Mean_Variance_Estimator___Gamma_Initial_Mean;
    float m_Mean_Variance_Estimator___Gamma_Initial_Variance;
    float m_Mean_Variance_Estimator__Gamma_Mean;
    float m_Mean_Variance_Estimator__Gamma_Variance;
    float m_Mean_Variance_Estimator___Uptime_Gamma;
    float m_Mean_Variance_Estimator___Uptime_Gating;
    float m_Mean_Variance_Estimator___Gating_Duration_Minutes;
    float m_Mean_Variance_Estimator___Sigmoid__K;
    float m_Mean_Variance_Estimator___Sigmoid__X0;
    float m_Mox_Model__Sraw_Std;
    float m_Mox_Model__Sraw_Mean;
    float m_Sigmoid_Scaled__K;
    float m_Sigmoid_Scaled__X0;
    float m_Sigmoid_Scaled__Offset_Default;
    float m_Adaptive_Lowpass__A1;
    float m_Adaptive_Lowpass__A2;
    bool m_Adaptive_Lowpass___Initialized;
    float m_Adaptive_Lowpass___X1;
    float m_Adaptive_Lowpass___X2;
    float m_Adaptive_Lowpass___X3;
} GasIndexAlgorithmParams;

/**
 * @brief 가스 지수 알고리즘 파라미터를 초기화하고 내부 상태를 리셋합니다.
 *
 * 지정된 알고리즘 타입(VOC 또는 NOx)에 맞게 파라미터를 설정합니다.
 * 프로그램 시작 시 한 번만 호출해야 합니다.
 *
 * @param params 알고리즘 파라미터 구조체의 포인터
 * @param algorithm_type 알고리즘 타입 (0: VOC, 1: NOx)
 */
void GasIndexAlgorithm_init(GasIndexAlgorithmParams* params,
                            int32_t algorithm_type);

/**
 * @brief 샘플링 간격을 지정하여 가스 지수 알고리즘을 초기화합니다.
 *
 * 기본 초기화 함수와 동일하지만, 측정 주기(샘플링 간격)를 직접 설정할 수 있습니다.
 *
 * @param params 알고리즘 파라미터 구조체의 포인터
 * @param algorithm_type 알고리즘 타입 (0: VOC, 1: NOx)
 * @param sampling_interval 알고리즘이 호출되는 샘플링 간격 (초 단위). 1초와 10초에 대해 테스트됨.
 */
void GasIndexAlgorithm_init_with_sampling_interval(
    GasIndexAlgorithmParams* params, int32_t algorithm_type,
    float sampling_interval);

/**
 * @brief 가스 지수 알고리즘의 내부 상태를 리셋합니다.
 *
 * 이전에 설정된 튜닝 파라미터는 유지됩니다.
 * 측정 중단 후 다시 시작할 때 호출합니다.
 *
 * @param params 알고리즘 파라미터 구조체의 포인터
 */
void GasIndexAlgorithm_reset(GasIndexAlgorithmParams* params);

/**
 * @brief 현재 알고리즘 상태를 가져옵니다.
 *
 * 반환된 값은 GasIndexAlgorithm_set_states() 함수에서 사용하여
 * 짧은 중단 후 초기 학습 단계를 건너뛰고 작동을 재개하는 데 사용할 수 있습니다.
 * 참고: 이 기능은 VOC 알고리즘 타입에서만, 그리고 최소 3시간 이상 연속 작동 후에만 사용할 수 있습니다.
 *
 * @param params 알고리즘 파라미터 구조체의 포인터
 * @param state0 저장할 상태 값 0
 * @param state1 저장할 상태 값 1
 */
void GasIndexAlgorithm_get_states(const GasIndexAlgorithmParams* params,
                                  float* state0, float* state1);

/**
 * @brief 이전에 가져온 알고리즘 상태를 설정하여 작동을 재개합니다.
 *
 * 이 기능은 10분 이상의 중단 후에는 사용해서는 안 됩니다.
 * GasIndexAlgorithm_init() 또는 GasIndexAlgorithm_reset() 호출 후에 한 번 호출합니다.
 * 참고: 이 기능은 VOC 알고리즘 타입에서만 사용할 수 있습니다.
 *
 * @param params 알고리즘 파라미터 구조체의 포인터
 * @param state0 복원할 상태 값 0
 * @param state1 복원할 상태 값 1
 */
void GasIndexAlgorithm_set_states(GasIndexAlgorithmParams* params, float state0,
                                  float state1);

/**
 * @brief 가스 지수 알고리즘을 사용자 정의하기 위한 파라미터를 설정합니다.
 *
 * GasIndexAlgorithm_init() 호출 후에 한 번 호출하여 기본값을 변경할 수 있습니다.
 *
 * @param params 알고리즘 파라미터 구조체의 포인터
 * @param index_offset 일반적인 (평균) 상태를 나타내는 가스 지수. 범위: 1..250, 기본값: VOC 100, NOx 1.
 * @param learning_time_offset_hours 오프셋의 장기 추정기 시간 상수. 과거 이벤트는 약 2배의 학습 시간 후에 잊혀집니다. 범위: 1..1000 [시간], 기본값: 12 [시간].
 * @param learning_time_gain_hours 게인의 장기 추정기 시간 상수. 범위: 1..1000 [시간], 기본값: 12 [시간]. (NOx 타입에서는 무관)
 * @param gating_max_duration_minutes 높은 가스 지수 신호 동안 추정기를 정지시키는 게이팅의 최대 지속 시간. 0 (게이팅 없음) 또는 1..3000 [분], 기본값: VOC 180분, NOx 720분.
 * @param std_initial 표준 편차의 초기 추정치. 낮은 값은 초기 학습 기간 동안 이벤트를 증폭시키지만, 장치 간 편차를 키울 수 있습니다. 범위: 10..5000, 기본값: 50. (NOx 타입에서는 무관)
 * @param gain_factor 가스 지수 계산 시 적용되는 게인 값을 조정하는 데 사용되는 팩터. 범위: 1..1000, 기본값: 230.
 */
void GasIndexAlgorithm_set_tuning_parameters(
    GasIndexAlgorithmParams* params, int32_t index_offset,
    int32_t learning_time_offset_hours, int32_t learning_time_gain_hours,
    int32_t gating_max_duration_minutes, int32_t std_initial,
    int32_t gain_factor);

/**
 * @brief 현재 설정된 튜닝 파라미터를 가져옵니다.
 *
 * 각 파라미터에 대한 설명은 GasIndexAlgorithm_set_tuning_parameters()를 참조하십시오.
 */
void GasIndexAlgorithm_get_tuning_parameters(
    const GasIndexAlgorithmParams* params, int32_t* index_offset,
    int32_t* learning_time_offset_hours, int32_t* learning_time_gain_hours,
    int32_t* gating_max_duration_minutes, int32_t* std_initial,
    int32_t* gain_factor);

/**
 * @brief 알고리즘에서 사용하는 샘플링 간격 파라미터를 가져옵니다.
 */
void GasIndexAlgorithm_get_sampling_interval(
    const GasIndexAlgorithmParams* params, float* sampling_interval);

/**
 * @brief 원시 센서 값으로부터 가스 지수 값을 계산합니다.
 *
 * 이 함수는 주기적으로(예: 매초) 호출되어야 합니다.
 *
 * @param params 알고리즘 파라미터 구조체의 포인터
 * @param sraw SGP4x 센서로부터의 원시 값
 * @param gas_index 계산된 가스 지수 값. 초기 블랙아웃 기간 동안은 0, 이후에는 1..500 사이의 값을 가집니다.
 */
void GasIndexAlgorithm_process(GasIndexAlgorithmParams* params, int32_t sraw,
                               int32_t* gas_index);

#endif /* GASINDEXALGORITHM_H_ */

#ifdef __cplusplus
}
#endif
