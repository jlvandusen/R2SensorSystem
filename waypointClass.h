class  waypointClass
{
    
  public:
    waypointClass(float pLong = 0, float pLat = 0)
      {
        fLong = pLong;
        fLat = pLat;
      }
      
    float getLat(void) {return fLat;}
    float getLong(void) {return fLong;}

  private:
    float fLong, fLat;
      
  
};  // waypointClass


// usage as array: 
//waypointClass myWaypoints[4] = {waypointClass(1,2), waypointClass(2,3), waypointClass(4,5), waypointClass() };

