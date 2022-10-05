class PIDClass
{
  public:
  PIDClass(){};
  PIDClass(double _kp,double _ki,double _kd):Kp(_kp),Ki(_ki),Kd(_kd){};
    double Kp;
    double Ki;
    double Kd;
    double pOut;
    double iOut;
    double dOut;
    double lastError;
    double lastCorrection;
    double divisor = 0.01;
    //if loop is constant time pass 1 into Dt but tune to it appropriately
    //this should strive to spit out a rate change term
    double getCorrectionTerm(long Dt,double error);
  private:
      
};