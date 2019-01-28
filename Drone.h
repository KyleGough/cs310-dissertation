class Drone {
public:
  float x;
  float y;
  float orientation;
  static void setParams(int _caveWidth, int _caveHeight);
  void setPosition(int _x, int _y);
  void sense();
private:
};
