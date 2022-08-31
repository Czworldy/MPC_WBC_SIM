#include "SupportPolygon.h"
#include "ros/ros.h"

SupportPolygon::SupportPolygon(int nSegment, int iterationsBetweenSEG):
    //_nSegment(nSegment),
    _iterationsBetweenSEG(iterationsBetweenSEG),
    _nSpline(nSegment){
        _iter_two_point = 0;
        _line_margin = paramf.line_margin;
    }

void SupportPolygon::SearchPolygon(const DMat<int>& SEGtable,
                                   const FootStateData & footContactCur,
                                   Vec2<float> * footholdsDes,
                                   int currentIteration,
                                   float line_margin){
    _line_margin = line_margin;
    cout << "line margin: " << endl;
    cout << _line_margin << endl;

    int curIter(currentIteration % _iterationsBetweenSEG);
    int curLeftSEG, curRightSEG;
    if(curIter){
        curRightSEG = _iterationsBetweenSEG - curIter;
        curLeftSEG = curIter;
    }
    else
    {
        curRightSEG = 0;
        curLeftSEG = 0;
    }

    //Start SupportPolygon
    _supportPolySet.clear();
    supportPolygonData startPolygon;
    for(size_t i=0; i<4; i++){
        if(footContactCur.isContact[i]){
           startPolygon.vertices.push_back(footContactCur.footLocation[i]);
        }

        //initialize for search
        _footLocationPre[i] = footContactCur.footLocation[i];
        _flag[i] = 0;
    }

    startPolygon.timeStart = curLeftSEG;

    if(startPolygon.vertices.size() == 2){
        Vec2<float> f_0, f_1;
        Vec2<float> point;
        f_0 << startPolygon.vertices[0];
        f_1 << startPolygon.vertices[1];



        startPolygon.vertices.clear();
        point << f_0[0] + (f_1[1] - f_0[1])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2,
                          f_0[1] + (f_0[0] - f_1[0])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2;
        startPolygon.vertices.push_back(point);
        point << f_1[0] + (f_1[1] - f_0[1])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2,
                          f_1[1] + (f_0[0] - f_1[0])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2;
        startPolygon.vertices.push_back(point);
        point << f_1[0] - (f_1[1] - f_0[1])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2,
                          f_1[1] - (f_0[0] - f_1[0])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2;
        startPolygon.vertices.push_back(point);
        point << f_0[0] - (f_1[1] - f_0[1])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2,
                          f_0[1] - (f_0[0] - f_1[0])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2;
        startPolygon.vertices.push_back(point);
        _iter_two_point++;
    }
    _supportPolySet.push_back(startPolygon);

    // ROS_INFO("_____________________POLY __________________________");
    // for(int i(0);i<startPolygon.vertices.size();i++){
    //     ROS_INFO_STREAM("\n"<<startPolygon.vertices[i]);
    // }
    // ROS_INFO_STREAM("time"<<startPolygon.timeStart);

    _isEvent = false;
    int nextSEG[4];
    
    for(int i=0; i<_nSpline; i++){
        for(int j=0; j<4; j++){
            if(i==(_nSpline-1)){
                _flag[j] = SEGtable(j, 0) - SEGtable(j,i);
                nextSEG[j] = SEGtable(j, 0);
            }
            else{
                _flag[j] = SEGtable(j, i+1) - SEGtable(j,i);
                nextSEG[j] = SEGtable(j, i);
            }
            if(_flag[j])
                _isEvent = true;
        }
        if((_isEvent)&&((i+1) != _nSpline)){
            supportPolygonData Polygon;
            for (int k=0; k<4; k++){
                if(_flag[k]==1){// A new contact.
                    Polygon.vertices.push_back(footholdsDes[k]);
                    _footLocationPre[k] = footholdsDes[k];
                }
                if((_flag[k]==0)&&(nextSEG[k]==1))
                    Polygon.vertices.push_back(_footLocationPre[k]);
            }
            Polygon.timeStart = _iterationsBetweenSEG * (i+1);
            //For trotting
            if(Polygon.vertices.size() == 2){
                Vec2<float> f_0, f_1;
                Vec2<float> point;
                f_0 << Polygon.vertices[0];
                f_1 << Polygon.vertices[1];

                Polygon.vertices.clear();
                point << f_0[0] + (f_1[1] - f_0[1])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2,
                                f_0[1] + (f_0[0] - f_1[0])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2;
                Polygon.vertices.push_back(point);
                point << f_1[0] + (f_1[1] - f_0[1])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2,
                                f_1[1] + (f_0[0] - f_1[0])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2;
                Polygon.vertices.push_back(point);
                point << f_1[0] - (f_1[1] - f_0[1])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2,
                                f_1[1] - (f_0[0] - f_1[0])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2;
                Polygon.vertices.push_back(point);
                point << f_0[0] - (f_1[1] - f_0[1])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2,
                                f_0[1] - (f_0[0] - f_1[0])/sqrt(pow((f_1[1] - f_0[1]),2) + pow((f_1[0] - f_0[0]),2))*_line_margin/2;
                Polygon.vertices.push_back(point);
            }
            _supportPolySet.push_back(Polygon);
            
            // ROS_INFO("_____________________POLY __________________________");
            // for(int i(0);i<Polygon.vertices.size();i++){
            //     ROS_INFO_STREAM("\n"<<Polygon.vertices[i]);
            // }
            // ROS_INFO_STREAM("time"<<Polygon.timeStart);

           _isEvent = false;
        }
    }

    _RecordData();
}

bool SupportPolygon::IntersectCheck(const vectorAligned<Vec2<float>> & polygon_1, 
                                    const vectorAligned<Vec2<float>> & polygon_2){
    Vec2<float> edge, projAxis;
    DVec<float> projPoly1(polygon_1.size()), projPoly2(polygon_2.size());
    //polygon_1
    for(int i(0); i<polygon_1.size(); i++){
        if(i != polygon_1.size()-1)
            edge = polygon_1[i+1] - polygon_1[i];
        else
            edge = polygon_1[0] - polygon_1[i];
        
        projAxis[0] = -edge[1];
        projAxis[1] = edge[0];
        
        // ROS_INFO_STREAM("_______________EDGE_________________\n" << edge);
        // ROS_INFO_STREAM("PROJECT_AXIS:\n" << projAxis);

        float m;
        for(int j(0);j<polygon_1.size(); j++){
            m = (projAxis[0]*polygon_1[j][0] + projAxis[1]*polygon_1[j][1])/(pow(projAxis[0],2)+pow(projAxis[1],2));
            projPoly1[j] = m;
            // ROS_INFO_STREAM("m_polygon1:   " <<polygon_1[j]);
            // ROS_INFO_STREAM(m);
        }
        for(int j(0); j<polygon_2.size(); j++){
            m = (projAxis[0]*polygon_2[j][0] + projAxis[1]*polygon_2[j][1])/(pow(projAxis[0],2)+pow(projAxis[1],2));
            projPoly2[j]=m;
            // ROS_INFO_STREAM("m_polygon2:   " <<polygon_2[j]);
            // ROS_INFO_STREAM(m);
        }

        bool isOverlap = _OverlapCheck(_MaxNMin(projPoly1), _MaxNMin(projPoly2));
         //ROS_INFO_STREAM("isOverLap:   " <<isOverlap);

        if(!isOverlap)
            return false;
    }

    //polygon_2
    for(int i(0); i<polygon_2.size(); i++){
        if(i != polygon_2.size()-1)
            edge = polygon_2[i+1] - polygon_2[i];
        else
            edge = polygon_2[0] - polygon_2[i];  
        
        projAxis[0] = -edge[1];
        projAxis[1] = edge[0];

        //ROS_INFO_STREAM("_______________EDGE_________________\n" << edge);
        //ROS_INFO_STREAM("PROJECT_AXIS:\n" << projAxis);
        
        float m;

        for(int j(0); j<polygon_1.size(); j++){
            m = (projAxis[0]*polygon_1[j][0] + projAxis[1]*polygon_1[j][1])/(pow(projAxis[0],2)+pow(projAxis[1],2));
            projPoly1[j] = m;
            
            //ROS_INFO_STREAM("m_polygon1:   " <<polygon_1[j]);
            //ROS_INFO_STREAM(m);
        }
        for(int j(0); j<polygon_2.size(); j++){
            m = (projAxis[0]*polygon_2[j][0] + projAxis[1]*polygon_2[j][1])/(pow(projAxis[0],2)+pow(projAxis[1],2));
            projPoly2[j] = m;

            //ROS_INFO_STREAM("m_polygon2:   " <<polygon_2[j]);
            //ROS_INFO_STREAM(m);
        }
        
        bool isOverlap = _OverlapCheck(_MaxNMin(projPoly1), _MaxNMin(projPoly2));

        //ROS_INFO_STREAM("isOverLap:   " <<isOverlap);

        if(!isOverlap)
            return false;
    }

    return true;
}

Vec2<float> SupportPolygon::_MaxNMin(DVec<float> projPoly){
    //ROS_INFO("________mm____");
    //for(int i(0); i<projPoly.size(); i++)
        //ROS_INFO_STREAM(projPoly[i]);
    
    for(int i(0); i<projPoly.rows()-1; i++){
        for(int j(0); j<projPoly.rows()-1-i; j++){
            if(projPoly[j]>projPoly[j+1]){
                std::swap(projPoly[j], projPoly[j+1]);
            }
        }
    }

    //ROS_INFO("MaxNMin");
    //for(int i(0); i<projPoly.rows(); i++)
        //ROS_INFO_STREAM(projPoly[i]);

    Vec2<float> maxNmin;
    maxNmin[0] = projPoly[0];//min
    maxNmin[1] = projPoly[projPoly.rows()-1];//max
    //ROS_INFO_STREAM("Sequence: \n"<<maxNmin);

    return maxNmin;
}

bool SupportPolygon::_OverlapCheck(const Vec2<float>& proj1,const Vec2<float> &proj2){
    bool result =  true;

    if((proj1[1]<=proj2[0])||(proj2[1]<=proj1[0]))
        result =false;//DO NOT HAVE OVERLAP
    
    return result;
}

vectorAligned<Vec31<double>>  SupportPolygon::getLineCoefficient(const vectorAligned<Vec2<float>>&  polygon,
                                                                 float  margin){
    Vec2<float> _edge, _normal;
    float _c;
    Vec2<float> _p0, _p1;
    Vec31<double> _line;

    _lineSet.clear();
    for(int i(0); i<polygon.size(); i++){
        if(i != polygon.size()-1){
            _edge = polygon[i+1] - polygon[i];
            _p1 = polygon[i+1];
            _p0 = polygon[i];
        }
        else{
            _edge = polygon[0] - polygon[i];
            _p1 = polygon[0];
            _p0 = polygon[i];
        }

        _normal[0] = -_edge[1];
        _normal[1] = _edge[0];

        _c = - _p1[0]*_p0[1] + _p0[0]*_p1[1] - margin;

        _line[0] = _normal[0];
        _line[1] = _normal[1];
        _line[2] = _c;
        //ROS_INFO_STREAM("LINE: \n"<< _line);
        _lineSet.push_back(_line);
    }
    return _lineSet;
}

Vec2<float> SupportPolygon::getFinalPolygonCenter(){
    int Index_final(_supportPolySet.size()-1);
    return getPolygonCenter(_supportPolySet[Index_final].vertices);
}

Vec2<float> SupportPolygon::getPolygonCenter(const vectorAligned<Vec2<float>>&  polygon){
    Vec2<float> center(0,0);
    if(polygon.size()>3){
        float det(0), tempDet(0);
        int j = 0, nVertice = polygon.size();
        for(int i(0); i<nVertice;i++){
            if(i+1 == nVertice)
                j = 0;
            else
                j = i+1;
            
            tempDet = polygon[i][0]*polygon[j][1] - polygon[j][0]*polygon[i][1];
            det += tempDet;

            center[0] +=(polygon[i][0] + polygon[j][0])*tempDet;
            center[1] +=(polygon[i][1] + polygon[j][1])*tempDet;
        }

        center[0] /= 3*det;
        center[1] /= 3*det;
    }
    else if(polygon.size()==3){
        for(int i(0); i<polygon.size(); i++){
            center[0] += 1/3. * polygon[i][0];
            center[1] += 1/3. * polygon[i][1];
        }
    }
    return center;
}

void SupportPolygon::_RecordData(){
    ofstream in;
    in.open("/home/MPC_WBC/dqwang/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/Motion_Plan/ResultData/SupportPolygon.txt", ios::trunc);

    for(int i(0); i<_supportPolySet.size(); i++){
        for(int j(0); j<_supportPolySet[i].vertices.size(); j++){
            in << _supportPolySet[i].vertices[j][0] << "\t" << _supportPolySet[i].vertices[j][1] << "\t";
        }
        in << "\n";
    }

    in.close();

    in.open("/home/MPC_WBC/dqwang/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/Motion_Plan/ResultData/SupportPolygon_Time.txt", ios::trunc);

    for(int i(0); i<_supportPolySet.size(); i++)
        in << _supportPolySet[i].timeStart << "\n";

    in.close();
}