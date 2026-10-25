#include <cassert>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdio>
#include "speed_drop_detector.h"
int main(int argc,char**argv) {
    assert(argc==2);
    std::ifstream in(argv[1]);std::string line;std::getline(in,line);
    SpeedDropDetector detector(SpeedDropDetector::Config{});
    float lastPwm=0;unsigned lastReverse=0;bool reversed=false;
    while(std::getline(in,line)) {
        for(char& c:line)if(c==',')c=' ';
        std::istringstream row(line);
        unsigned index,us;float angle,speed,raw,pwm,drop,base,target;
        if(!(row>>index>>us>>angle>>speed>>raw>>pwm>>drop>>base>>target))continue;
        float ceiling=std::fmax(40.f,target*1.15f);
        float trigger=reversed && us-lastReverse<=6000 ? std::fmax(40.f,target*.25f):INFINITY;
        if(detector.update(angle,us,true,true,ceiling,trigger,speed)) {
            std::printf("%u %.3f %.3f\n",us,angle,speed);return 0;
        }
        if(pwm<0){reversed=true;lastReverse=us;}
        lastPwm=pwm;
    }
    return 2;
}
