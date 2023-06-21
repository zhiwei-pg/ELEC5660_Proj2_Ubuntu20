/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
#include "board.h"
#include <fstream>
using namespace std;
using namespace cv;
namespace old_aruco
{

/**
*
*
*/
BoardConfiguration::BoardConfiguration()
{
    mInfoType=NONE;
}
/**
*
*
*/
BoardConfiguration::BoardConfiguration ( const BoardConfiguration  &T ): vector<MarkerInfo>(T)
{
//     MarkersInfo=T.MarkersInfo;
    mInfoType=T.mInfoType;
}

/**
*
*
*/
BoardConfiguration & BoardConfiguration ::operator=(const BoardConfiguration  &T) {
//     MarkersInfo=T.MarkersInfo;
    vector<MarkerInfo>::operator=(T);
    mInfoType=T.mInfoType;
    return *this;
}
/**
*
*
*/
void BoardConfiguration::saveToFile ( string sfile ) throw ( cv::Exception )
{

    cv::FileStorage fs ( sfile,cv::FileStorage::WRITE );
    saveToFile(fs);

}
/**Saves the board info to a file
*/
void BoardConfiguration::saveToFile(cv::FileStorage &fs)throw (cv::Exception) {
    fs<<"aruco_bc_nmarkers"<< ( int ) size();
    fs<<"aruco_bc_mInfoType"<< ( int ) mInfoType;
    fs<<"aruco_bc_markers"<<"[";
    for ( size_t i=0;i<size();i++ )
    {
        fs << "{:" << "id" << at(i).id ;

        fs<<"corners"<< "[:";
        for (int c=0;c<at(i).size();c++)
            fs<<at(i)[c];
        fs<<"]";
        fs <<  "}";
    }
    fs << "]";
}

/**
*
*
*/
void BoardConfiguration::readFromFile ( string sfile ) throw ( cv::Exception )
{
    cv::FileStorage fs ( sfile,cv::FileStorage::READ );
    readFromFile(fs);

}


/**Reads board info from a file
*/
void BoardConfiguration::readFromFile(cv::FileStorage &fs)throw (cv::Exception)
{
    int aux=0;
    //look for the nmarkers
    if ( fs["aruco_bc_nmarkers"].name() !="aruco_bc_nmarkers" )
        throw cv::Exception ( 81818,"BoardConfiguration::readFromFile","invalid file type" ,__FILE__,__LINE__ );
    fs["aruco_bc_nmarkers"]>>aux;
    resize ( aux );
    fs["aruco_bc_mInfoType"]>>mInfoType;
    cv::FileNode markers=fs["aruco_bc_markers"];
    int i=0;
    for (FileNodeIterator it = markers.begin();it!=markers.end();++it,i++) {
        at(i).id=(*it)["id"];
        FileNode FnCorners=(*it)["corners"];
        for (FileNodeIterator itc = FnCorners.begin();itc!=FnCorners.end();++itc) {
            vector<float> coordinates3d;
            (*itc)>>coordinates3d;
            if(coordinates3d.size()!=3)
	       throw cv::Exception ( 81818,"BoardConfiguration::readFromFile","invalid file type 3" ,__FILE__,__LINE__ );
	    cv::Point3f point(coordinates3d[0],coordinates3d[1],coordinates3d[2]);
            at(i).push_back(point);
        }
    }
}

/**
 */
int BoardConfiguration::getIndexOfMarkerId(int id)const
{
 
 for(size_t i=0;i<size();i++)
   if( at(i).id==id)return i;
 return -1;   
}

/**
 */
const MarkerInfo& BoardConfiguration::getMarkerInfo(int id)const throw (cv::Exception)
{
 for(size_t i=0;i<size();i++)
   if( at(i).id ==id) return at(i);
 throw cv::Exception(111,"BoardConfiguration::getMarkerInfo","Marker with the id given is not found",__FILE__,__LINE__);
  
}
    /** 
 */
void BoardConfiguration::getIdList(std::vector< int >& ids, bool append) const
{
 if (!append) ids.clear();
 for(size_t i=0;i<size();i++)
   ids.push_back(at(i).id);
}
};
