#ifndef GlobalParams_H_
#define GlobalParams_H_
#include <iostream>

typedef enum
{
    MATCHING = 1,
    TRACKING = 2
} MatchingType;

class GlobalParams
{
  public:
    GlobalParams()
    {
        m_float_fast_threshold = 10.0f;
    }
    ~GlobalParams()
    {
    }
    float getFastThreshold()
    {
        return m_float_fast_threshold;
    }
    void setFastThreshold(float v){
        m_float_fast_threshold=v;
    }
    MatchingType getMatchingType()
    {
        return m_matchingtype;
    }
    void setMatchingType(MatchingType m)
    {
        m_matchingtype = m;
    }
    int getMinimumMaintainedTrackedFeatureNumber()
    {
        return m_int_minimum_maintained_tracked_feature_numbers;
    }
    void setMinimumMaintainedTrackedFeatureNumber(int v)
    {
        m_int_minimum_maintained_tracked_feature_numbers = v;
    }

  private:
    float m_float_fast_threshold;
    MatchingType m_matchingtype;
    int m_int_minimum_maintained_tracked_feature_numbers;
};

#endif