#include <signal/controllers/motorcontroller.hpp>

namespace signal{
    

namespace controllers{
    /**
     * @brief Construct a new CMotorController::CMotorController object
     * 
     * @param f_encoder Reference to the encoder getter interface.
     * @param f_pid     Reference to the controller interface.
     * @param f_converter [Optional] Pointer to the converter interface. 
     * @param f_inf_dist   [Optional] Inferior limit of reference signal.
     * @param f_sup_dist   [Optional] Superior limit of reference signal.
     * @param f_inf_ref   [Optional] Inferior limit of reference signal.
     * @param f_sup_ref   [Optional] Superior limit of reference signal.
     */
    CMotorController::CMotorController(hardware::encoders::IEncoderGetter&          f_encoder
                            ,ControllerType<double>&                                f_pid
                            ,signal::controllers::IConverter*                       f_converter
                            ,float                                                  f_inf_dist
                            ,float                                                  f_sup_dist
                            ,float                                                  f_inf_ref
                            ,float                                                  f_sup_ref)
        :m_encoder(f_encoder)
        ,m_pid(f_pid)
        ,m_converter(f_converter)
        ,m_nrHighPwm(0)
        ,m_maxNrHighPwm(10)
        ,m_control_sup(0.5)
        ,m_control_inf(-0.5)
        ,m_ref_abs_inf(10.0)
        ,m_mes_abs_inf(30.0)
        ,m_mes_abs_sup(300.0)
        ,m_inf_dist(f_inf_dist)
        ,m_sup_dist(f_sup_dist)
        ,m_inf_ref(f_inf_ref)
        ,m_sup_ref(f_sup_ref)
    {
    }

    /**
     * @brief Set the reference signal value.
     * 
     * @param f_RefRps The value of the reference signal
     */
    void CMotorController::setRef(double f_RefRps)
    {
        m_RefRps=f_RefRps;
    }

    /**
     * @brief Set the reference signal value.
     * 
     * @param f_RefDist The value of the reference signal
     */
    void CMotorController::setDist(double f_RefDist)
    {
        m_RefDist=f_RefDist;
        m_encoder.startDistMeasure();
    }

    /** @brief  Get the value of the control signal calculated last time.
     * 
     */
    double CMotorController::getSpeed()
    {
        return m_u;
    }

    /** @brief  Clear the memory of the controller. 
     *
     */
    void CMotorController::clearSpeed()
    {
        m_pid.clear();
    }

    /**
     * @brief It checks if the distance has been covered.
     * 
     * @return true control works fine
     * @return false appeared an error
     */
    int8_t CMotorController::doneDist()
    {
        double dist = m_encoder.getTraveledDistance();

        if ((m_RefDist > 0 && m_RefDist >= dist) || (m_RefDist < 0 && m_RefDist <= dist))
        {
            return 1;
        }
        else
        {
            return -1;
        }
    }

    /** @brief  Clear the memory of the distance. 
     *
     */
    void CMotorController::clearDist()
    {
        m_encoder.stopDistMeasure();
    }

    /**
     * @brief It calculates the next value of the control signal, by utilizing the given interfaces.
     * 
     * @return true control works fine
     * @return false appeared an error
     */
    int8_t CMotorController::control()
    {
        // Mesurment speed value
        float  l_MesRps = m_encoder.getSpeedRps();
        bool   l_isAbs = m_encoder.isAbs();
        float  l_ref;

        // Check the measured value and the superior limit for avoiding over control state.
        // In this case deactivate the controller. 
        if(std::abs(l_MesRps) > m_mes_abs_sup) {
            m_RefRps = 0.0;
            m_u = 0.0;
            return -1;
        }
        // Check the inferior limits of reference signal and measured signal for standing state.
        // Inactivate the controller to not brake the robot, when it stopped. 
        if(std::abs(m_RefRps) < m_ref_abs_inf && std::abs(l_MesRps) < m_mes_abs_inf ){
            m_u = 0.0;
            return 1; 
        }

        // Check measured value is orientated or absolute
        if ( l_isAbs ){
            l_ref = std::abs(m_RefRps);
        } else { 
            l_ref = m_RefRps;
        }
        float l_error=l_ref-l_MesRps;
        float l_v_control = m_pid.calculateControl(l_error);
        float l_pwm_control = converter(l_v_control);
        

        // Verify the number of high control signal and the measued rotation speed. When it's true, than the encoder doesn't measure the correct rotation speed,
        // so the calculated control signal has a too high value. 
        if(m_nrHighPwm>m_maxNrHighPwm && l_MesRps==0){
            m_pid.clear();
            m_RefRps = 0.0;
            m_u = 0.0;
            m_nrHighPwm = 0;
            return -2;
        }

        m_u=l_pwm_control;
        
        // Check measured value is oriantated or absolute.
        if(m_RefRps<0 && l_isAbs)
        {
            m_u=m_u*-1.0;
        }
        return 1;
    }

    /** @brief  
     *
     * Apply the converter interface to change the measurment unit.
     *
     * @param f_u                  Input control signal
     * @return                     Converted control signal
     */
    double CMotorController::converter(double f_u)
    {
        double l_pwm=f_u;
        // Convert the control signal from V to PWM
        if(m_converter!=NULL){
            l_pwm = (*m_converter)(f_u);
        }

        // Check the pwm control signal and limits
        if( l_pwm < m_control_inf ){
            l_pwm  = m_control_inf;
            ++m_nrHighPwm;
        } else if( l_pwm > m_control_sup ){
            l_pwm  = m_control_sup;
            ++m_nrHighPwm;
        }
        else{
            m_nrHighPwm = 0;
        }

        return l_pwm;
    }

    /**
     * @brief It verifies whether a number is in a given range
     * 
     * @param f_RefRps reference value for controller in rotation per second  
     * @return true means, that the value is in the range
     * @return false means, that the value isn't in the range
     */
    bool CMotorController::inRange(double f_RefRps){
        return m_inf_ref<=f_RefRps && f_RefRps<=m_sup_ref;
    }

    /**
     * @brief It checks whether a number is in a given range
     * 
     * @param f_RefDist value 
     * @return true means, that the value is in the range
     * @return false means, that the value isn't in the range
     */
    bool CMotorController::distInRange(double f_RefDist){
        return m_inf_dist<=f_RefDist && f_RefDist <=m_sup_dist;
    }

}; //  namespace controllers
};// namespace signal