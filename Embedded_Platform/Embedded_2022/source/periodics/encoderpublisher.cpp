#include <periodics/encoderpublisher.hpp>

namespace periodics
{

    /** \brief CEncoderPublisher contructor
     *
     *  It initializes the member parameter, the publisher is initially deactivated.
     *
     *  @param f_period       period value
     *  @param f_encoder      reference to encoder object
     *  @param f_serial       reference to the serial object
     */
    CEncoderPublisher::CEncoderPublisher(uint32_t            f_period
                                            ,hardware::encoders::IEncoderGetter&     f_encoder
                                            ,BufferedSerial&             f_serial)
        :utils::task::CTask(f_period)
        ,m_isActive(false)
        ,m_encoder(f_encoder)
        ,m_serial(f_serial)
    {
    }

    
    /** \brief  Serial callback method to activate or deactivate the publisher. 
     * When the received integer value is bigger or equal to 1, then the publisher become 
     * active and send messages, otherwise is deactivated. 
     *
     * @param a                   input received string
     * @param b                   output reponse message
     * 
     */
    void CEncoderPublisher::serialCallbackENCODERPUBcommand(char const * a, char * b) {
        int l_isActivate=0;
        uint32_t l_res = sscanf(a,"%d",&l_isActivate);
        if(l_res==1){
            m_isActive=(l_isActivate>=1);
            sprintf(b,"ack;;");
        }else{
            sprintf(b,"sintax error;;");
        }
    }

    /** \brief It's periodically applied method to send message to other device. 
     */
    void CEncoderPublisher::_run()
    {
        if(!m_isActive) return;
        float l_rps = m_encoder.getSpeedRps();
        printf("@5:%.2f;;\r\n",l_rps);  
    }                        

}; // namespace periodics
