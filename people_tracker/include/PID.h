// #ifndef FGM_PLUGIN_PID_H_
// #define FGM_PLUGIN_PID_H_

class PIDController
{
public:
    PIDController();

    double computeControlSignal(double);
    double derivativeFilteredControlSignal(double);

    // Setters
    void setPIDCoeffs(double, double, double);
    void setIntegratorSaturations(double, double);
    void setOutputSaturations(double, double);
    void setSampleTime(double);
    void setDerivativeFilterConstant(double);

    // Getters
    double getControllerOutput() const;
    double getSampleTime() const;

private:
    // Controller gains
    double Kp_;
    double Ki_;
    double Kd_;

    // Derivative filter constant
    double N_;

    // Integrator saturations
    double integratorSatLower_;
    double integratorSatUpper_;

    // Output saturations
    double outputSaturationLower_;
    double outputSaturationUpper_;

    // Sample time(in seconds)
    double T_;

    // PID lines output
    double proportional_;
    double integrator_;
    double differentiator_;

    // Classical PID variable(s)
    double prevError_;

    // Derivative filtered discrete PID variable(s)
    double e0_, e1_, e2_;
    double y0_, y1_, y2_;

    // Controller output
    double output_;    
};

#endif