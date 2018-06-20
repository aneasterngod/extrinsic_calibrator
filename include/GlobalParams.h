#ifndef GlobalParams_H_
#define GlobalParams_H_
#include <iostream>


typedef enum{
    MATCHING = 1,
    TRACKING = 2
}MatchingType;

class GlobalParams{
public:
    GlobalParams(){
        m_float_fast_threshold = 10.0f;
    }
    ~GlobalParams(){

    }
    float getFastThreshold(){
        return m_float_fast_threshold;
    }
    MatchingType getMatchingType(){
        return m_matchingtype;
    }
    void setMatchingType(MatchingType m){
        m_matchingtype = m;
    }
private:
    float m_float_fast_threshold;
    MatchingType m_matchingtype;
};

#endif