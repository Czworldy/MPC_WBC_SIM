#　X20 simulation test

计算相机内参的公式

+ reference:http://playerstage.sourceforge.net/wiki/GazeboProblemResolutionGuide

+ ```
  vfov = 2*atan(height/(2*flength))
  hfov = 2*atan(width/(2*flength))
  f = (width/2) / tan( deg2rad(hfov)/2) 
  ```

 f_x = (720/2)/tan(1.0467/2) ,f_y = (480/2)/tan(1.0467/2), cx =360.5, cy=240.5

