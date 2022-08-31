#ifndef SUPPORTPOLYGON_H
#define SUPPORTPOLYGON_H

#include "cppTypes.h"
#include "Gait.h"
#include "UserParameter.h"
#include "ros/ros.h"
#include <fstream>

struct supportPolygonData 
{
    vectorAligned<Vec2<float>> vertices; //the x-y coordinates of the feet which are in contact
    int timeStart; //the start of the polygon//
};

class SupportPolygon{
    public:
        SupportPolygon(int nSegment, int iterationsBetweenSEG);
        ~SupportPolygon(){};

        void SearchPolygon(const DMat<int> & SEGtable,
                           const FootStateData & footContactCur,
                           Vec2<float> * footholdsDes,
                           int currentIteration,
                           float line_margin);
        std::vector<supportPolygonData> getSupportPolySet(){return _supportPolySet;}
        bool IntersectCheck(const vectorAligned<Vec2<float>> & polygon_1,  
                            const vectorAligned<Vec2<float>> & polygon_2);
        vectorAligned<Vec31<double>>  getLineCoefficient(const vectorAligned<Vec2<float>>&  polygon,
                                                         float  margin);//a,b,c
        Vec2<float> getPolygonCenter(const vectorAligned<Vec2<float>>&  polygon);
        Vec2<float> getFinalPolygonCenter();

    protected:
        Vec2<float> _MaxNMin(DVec<float> projPoly);
        bool _OverlapCheck(const Vec2<float>& proj1, const Vec2<float>& proj2);

        void _RecordData();

        std::vector<supportPolygonData> _supportPolySet;
        vectorAligned<Vec31<double>> _lineSet;

        //int _nSegment;
        int _iterationsBetweenSEG;
        int _nSpline;

        int _flag[4];
        Vec2<float> _footLocationPre[4];
        bool _isEvent;    
        UserParameter<float> paramf;
        float _line_margin;
        long long int _iter_two_point;

};

#endif

