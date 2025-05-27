#include <hardware/drivers/steeringmotor.hpp>


namespace hardware::drivers{
    /** @brief  It initializes the pwm parameters and set the steering in zero position. Initially it sets the limits of input degree values to -23 and +23 degree. 
     *
     *  @param f_pwm               pin connected to servo motor
     */
    CSteeringMotor::CSteeringMotor(PinName f_pwm)
        :CSteeringMotor(f_pwm,-23.0,23.0)
        
    {
    };

    /**
     * @brief It initializes the pwm parameters and it sets the steering in zero position, the limits of the input degree value.
     * 
     * @param f_pwm               pin connected to servo motor
     * @param f_inf_limit         inferior limit 
     * @param f_sup_limit         superior limit
     * 
     */
    CSteeringMotor::CSteeringMotor(PinName f_pwm, float f_inf_limit, float f_sup_limit)
        :m_pwm(f_pwm)
        ,m_inf_limit(f_inf_limit)
        ,m_sup_limit(f_sup_limit)
    {
        m_pwm.period_ms(20); 
        // Set position to zero   
        m_pwm.write(0.07525);
    };


    /** @brief  CSteeringMotor class destructor
     */
    CSteeringMotor::~CSteeringMotor()
    {
    };

    /** @brief  It modifies the angle of the servo motor, which controls the steering wheels. 
     *
     *  @param f_angle      angle degree, where the positive value means right direction and negative value the left direction. 
     */
    void CSteeringMotor::setAngle(float f_angle)
    {
        printf("setAngle: %.3f, pwm value: %.3f\r\n", f_angle, conversion(f_angle));
        // m_pwm.write(conversion(f_angle));
        m_pwm.write(f_angle);
    };

    /** @brief  It converts angle degree to duty cycle for pwm signal. 
     * 
     *  @param f_angle    angle degree
     *  \return         duty cycle in interval [0,1]
     */
    float CSteeringMotor::conversion(float f_angle)
    {
        return (0.0009505 * f_angle + 0.07525);
    };

    /**
     * @brief It verifies whether a number is in a given range
     * 
     * @param f_angle value 
     * @return true means, that the value is in the range
     * @return false means, that the value isn't in the range
     */
    bool CSteeringMotor::inRange(float f_angle){
        return m_inf_limit<=f_angle && f_angle<=m_sup_limit;
    };
}; // namespace hardware::drivers