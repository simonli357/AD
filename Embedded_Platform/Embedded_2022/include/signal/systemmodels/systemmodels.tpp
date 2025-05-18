/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

#ifndef SYSTEM_MODELS_TPP
#define SYSTEM_MODELS_TPP

#ifndef SYSTEM_MODELS_HPP
#error __FILE__ should only be included from systemmodels.hpp.
#endif // SYSTEM_MODELS_HPP



/******************************************************************************/
/**
 * @brief Construct a new CDiscreteTransferFunction object without input, it initializes the coefficients with zero, only the first coefficient with 1 value. 
 * 
 * @tparam T        Type of the variables
 * @tparam NNum     Order of the nominator polynomial 
 * @tparam NDen     Order of the denominator polynomial 
 */
template <class T,uint32_t NNum,uint32_t NDen>
signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,NNum,NDen>::CDiscreteTransferFunction()
    :m_num(CNumType::zeros())
    ,m_den(CDenModType::zeros())
    ,m_denCoef(1)
    ,m_memInput()
    ,m_memOutput()
{
}


/**
 * @brief Construct a new CDiscreteTransferFunction object with input polynomials 
 * 
 * @tparam T        Type of the variables
 * @tparam NNum     Order of the nominator polynomial 
 * @tparam NDen     Order of the denominator polynomial
 * @param f_num     Nominator polynomial coefficients
 * @param f_den     Denominator polynomial coefficients
 */
template <class T,uint32_t NNum,uint32_t NDen>
signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,NNum,NDen>::CDiscreteTransferFunction(const CNumType& f_num,const CDenType& f_den)
    :m_num(CNumType::zeros())
    ,m_den(CDenModType::zeros())
    ,m_denCoef(1)
    ,m_memInput()
    ,m_memOutput()
{
    this->setNum(f_num);
    this->setDen(f_den);
}


/** \brief  Clear memory
 *
 *  Set all memories to zero.  
 *  
 */
template <class T,uint32_t NNum,uint32_t NDen>
void signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,NNum,NDen>::clearMemmory()
{
    m_memInput=CInputMem::zeros();
    m_memOutput=COutputMem::zeros();
}

/** \brief  Applying the transfer function on the input signal value
 *
 *  @param f_input  next input signal value
 *  @return  the calculated next output value. 
 */
template <class T,uint32_t NNum,uint32_t NDen>
T signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,NNum,NDen>::operator()(const T& f_input)
{
    shiftMemory<NNum>(m_memInput);
    m_memInput[0][0]=f_input;
    T l_output=((m_memInput*m_num-m_memOutput*m_den)[0][0])/m_denCoef;
    shiftMemory<NDen-1>(m_memOutput);
    m_memOutput[0][0]=l_output;
    return l_output;
}

/** \brief  Setting nominator coefficients
 *
 *  @param f_num    nominator coefficients
 *  
 */
template <class T,uint32_t NNum,uint32_t NDen>
void signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,NNum,NDen>::setNum(const CNumType& f_num)
{
    for(uint32_t i=0;i<NNum;++i)
    {
        m_num[i][0]=f_num[i][0];
    }
}

/** \brief  Setting denominator coefficients 
 *
 *  @param f_den    denominator coefficients
 *  
 */
template <class T,uint32_t NNum,uint32_t NDen>
void signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,NNum,NDen>::setDen(const CDenType& f_den)
{
    for(uint32_t i=1;i<NDen;++i)
    {
        m_den[i-1][0]=f_den[i][0];
    }
    m_denCoef = f_den[0][0];
}

/** \brief  Getting the last calculated output
 *
 *  
 *  @return last output value
 */
template <class T,uint32_t NNum,uint32_t NDen>
T signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,NNum,NDen>::getOutput()
{
    return m_memOutput[0][0];
}

/** \brief  Shift the memorized values. After the shifting the first element in the memory can be placed with a new one.
 *
 *  @param f_mem           memory matrix.
 *  @return   none
 */
template <class T,uint32_t NNum,uint32_t NDen>
template<uint32_t N>
void signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,NNum,NDen>::shiftMemory(utils::linalg::CMatrix<T,1,N>& f_mem)
{
    for(uint32_t i=N-1;i>0;--i)
    {
        f_mem[0][i]=f_mem[0][i-1];
    }
}

/** \brief  Getter nominator 
 *
 *  @return the coefficients of the nominator 
 */
template <class T,uint32_t NNum,uint32_t NDen>
const typename signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,NNum,NDen>::CNumType& signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,NNum,NDen>::getNum(){
    return m_num;
}

/** \brief  Getter denominator
 * Return the denominator without the first coefficient.
 * 
 *  @return  the coefficients of denominator
 */
template <class T,uint32_t NNum,uint32_t NDen>
const typename signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,NNum,NDen>::CDenModType& signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,NNum,NDen>::getDen(){
    return m_den;
}  

/** \brief  Getter denomitor first coefficeint
 *
 *  @return   value of the denomitor first coefficeint
 */
template <class T,uint32_t NNum,uint32_t NDen>
float signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,NNum,NDen>::getDenCurrent(){
    return m_denCoef;
}


/******************************************************************************/
/** @brief  CSSModel Class constructor
 *
 *  Initialize the model matrixes of the state space model based the given input parameters.
 *  The initial state is zero and the control-observation model is a zero matrix. 
 *
 *  @param f_stateTransitionMatrix  state-state transition model
 *  @param f_inputMatrix            control-state transition model
 *  @param f_measurementMatrix      state-observation transition model
 */
template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
signal::systemmodels::lti::mimo::CSSModel<T,NA,NB,NC>::CSSModel(
        const CStateTransitionType& f_stateTransitionMatrix,
        const CInputMatrixType& f_inputMatrix,
        const CMeasurementMatrixType& f_measurementMatrix) 
    : m_stateVector()
    , m_stateTransitionMatrix(f_stateTransitionMatrix)
    , m_inputMatrix(f_inputMatrix)
    , m_measurementMatrix(f_measurementMatrix)
    , m_directTransferMatrix()
{
}

/** @brief  CSSModel Class constructor
 *
 *  Initialize the model matrixes of the state space model based the given input parameters.
 *  The initial state is zero. 
 *
 *  @param f_stateTransitionMatrix  state-state transition model
 *  @param f_inputMatrix            control-state transition model
 *  @param f_measurementMatrix      state-observation transition model
 *  @param f_directTransferMatrix   control-observation transition model
 */
template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
signal::systemmodels::lti::mimo::CSSModel<T,NA,NB,NC>::CSSModel(
        const CStateTransitionType& f_stateTransitionMatrix,
        const CInputMatrixType& f_inputMatrix,
        const CMeasurementMatrixType& f_measurementMatrix,
        const CDirectTransferMatrixType& f_directTransferMatrix) 
    : m_stateVector()
    , m_stateTransitionMatrix(f_stateTransitionMatrix)
    , m_inputMatrix(f_inputMatrix)
    , m_measurementMatrix(f_measurementMatrix)
    , m_directTransferMatrix(f_directTransferMatrix)
{
}

/** @brief  CSSModel Class constructor
 *
 *  Initialize the model matrixes of the state space model based the given input parameters.
 *
 *  @param f_stateTransitionMatrix  state-state transition model
 *  @param f_inputMatrix            control-state transition model
 *  @param f_measurementMatrix      state-observation transition model
 *  @param f_directTransferMatrix   control-observation transition model
 *  @param f_state                  initial state of the system model
 */
template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
signal::systemmodels::lti::mimo::CSSModel<T,NA,NB,NC>::CSSModel(
        const CStateTransitionType& f_stateTransitionMatrix,
        const CInputMatrixType& f_inputMatrix,
        const CMeasurementMatrixType& f_measurementMatrix,
        const CDirectTransferMatrixType& f_directTransferMatrix,
        const CStateType& f_state) 
    : m_stateVector(f_state)
    , m_stateTransitionMatrix(f_stateTransitionMatrix)
    , m_inputMatrix(f_inputMatrix)
    , m_measurementMatrix(f_measurementMatrix)
    , m_directTransferMatrix(f_directTransferMatrix)
{
}

/** @brief  Apply the model with the input control parameter and returning the observation.
  *
  * @param f_inputVector        control values
  * @return                     observation values
  */
template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
utils::linalg::CColVector<T,NC> signal::systemmodels::lti::mimo::CSSModel<T,NA,NB,NC>::operator()(const CControlType& f_inputVector)
{
    updateState(f_inputVector);
    return getOutput(f_inputVector);
}

/** @brief  Update state of the model based the input control parameter.
  *
  * @param f_inputVector        control values
  */
template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
void signal::systemmodels::lti::mimo::CSSModel<T,NA,NB,NC>::updateState(const CControlType& f_inputVector)
{
    m_stateVector = m_stateTransitionMatrix * m_stateVector + m_inputMatrix * f_inputVector;
}

/** @brief  Calculate and return the observation values based on the control value and the state of the system.
  *
  * @param f_inputVector        control values
  * @return                     observation values
  */
template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
utils::linalg::CColVector<T,NC> signal::systemmodels::lti::mimo::CSSModel<T,NA,NB,NC>::getOutput(const CControlType& f_inputVector)
{
    return m_measurementMatrix * m_stateVector + m_directTransferMatrix * f_inputVector;
}




/******************************************************************************/
/** \brief  CDiscreteTimeSystemModel class constructor for non-linear systems
 *
 *  @param f_dt      Sampling time 
 */
template <class T,uint32_t NA, uint32_t NB,uint32_t NC>
signal::systemmodels::nlti::mimo::CDiscreteTimeSystemModel<T,NA,NB,NC>::CDiscreteTimeSystemModel(const double f_dt)
    : m_states()
    , m_dt(f_dt)
{
}

/** \brief  CDiscreteTimeSystemModel class constructor for non-linear systems
 *
 *  @param f_states  Initial states of the system
 *  @param f_dt      Sampling time
 */
template <class T,uint32_t NA, uint32_t NB,uint32_t NC>
signal::systemmodels::nlti::mimo::CDiscreteTimeSystemModel<T,NA,NB,NC>::CDiscreteTimeSystemModel(
        const CStatesType&     f_states
       ,const double           f_dt)
    : m_states(f_states)
    , m_dt(f_dt)
{
}




#endif // SYSTEM_MODELS_TPP

