#pragma once
#include <cstdint>
constexpr int ESP_OK=0;
constexpr int PCNT_UNIT_0=0, PCNT_CHANNEL_0=0, PCNT_CHANNEL_1=1;
constexpr int PCNT_COUNT_INC=1, PCNT_COUNT_DEC=-1;
constexpr int PCNT_MODE_KEEP=1, PCNT_MODE_REVERSE=-1;
struct pcnt_config_t {
    int unit,channel,pulse_gpio_num,ctrl_gpio_num,pos_mode,neg_mode;
    int hctrl_mode,lctrl_mode;
    int16_t counter_h_lim,counter_l_lim;
};
extern pcnt_config_t fakeChannels[2];
extern int16_t fakePcnt;
extern bool fakePcntFailure;
inline int pcnt_unit_config(const pcnt_config_t* c) {
    fakeChannels[c->channel]=*c; return fakePcntFailure ? -1 : ESP_OK;
}
inline int pcnt_counter_pause(int) { return ESP_OK; }
inline int pcnt_counter_resume(int) { return ESP_OK; }
inline int pcnt_counter_clear(int) { fakePcnt=0; return ESP_OK; }
inline int pcnt_set_filter_value(int,int) { return ESP_OK; }
inline int pcnt_filter_enable(int) { return ESP_OK; }
inline int pcnt_get_counter_value(int,int16_t* value) {
    *value=fakePcnt; return fakePcntFailure ? -1 : ESP_OK;
}
