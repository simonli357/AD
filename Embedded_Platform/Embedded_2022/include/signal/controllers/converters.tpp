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



#ifndef CONVERTERS_TPP
#define CONVERTERS_TPP


#ifndef CONVERTERS_H
#error __FILE__ should only be included from converters.hpp.
#endif // CONVERTERS_H

/**
 * @brief Construct a new CConverterPolynom<NOrd>::CConverterPolynom object
 * 
 * @param f_coeff The list of the coeffience.
 */
template<uint8_t NOrd>
CConverterPolynom<NOrd>::CConverterPolynom(std::array<float,NOrd+1> f_coeff)
:m_coeff(f_coeff)
{
}


/**
 * @brief Convert the input value. 
 * 
 */
template<uint8_t NOrd>
float CConverterPolynom<NOrd>::operator()(float f_v){
    float l_res = m_coeff[NOrd];
    for (uint8_t i = 0;i<NOrd;++i){
        l_res += std::pow(f_v, NOrd-i)*m_coeff[i];
    }
    return l_res;
}


/**
 * @brief Construct a new CConverterSpline<NrBreak, NOrd>::CConverterSpline object
 * 
 * @tparam NrBreak 
 * @tparam NOrd 
 * @param f_breaks The list of the break points.
 * @param f_splines The list of the polynomial function.
 */
template <uint8_t NrBreak, uint8_t NOrd>
CConverterSpline<NrBreak, NOrd>::CConverterSpline(CConverterSpline<NrBreak, NOrd>::CBreakContainerType f_breaks,CConverterSpline<NrBreak, NOrd>::CSplineContainerType f_splines)
:m_breaks(f_breaks)
,m_splines(f_splines)
{
}

/**
 * @brief Convert the input value.
 * 
 */
template <uint8_t NrBreak, uint8_t NOrd>
float CConverterSpline<NrBreak, NOrd>::operator()(float f_value){
    typename CConverterSpline<NrBreak, NOrd>::CBreakContainerType::iterator it_breaks0=NULL,it_breaks1=NULL;
    typename CConverterSpline<NrBreak, NOrd>::CSplineContainerType::iterator it_spline = m_splines.begin();
    for (it_breaks1 = m_breaks.begin();it_breaks1 != m_breaks.end();){
        if((it_breaks0==NULL || (it_breaks0!=NULL && (*it_breaks0)<f_value)) && f_value<=(*it_breaks1)){
            break;
        }
        it_breaks0 = it_breaks1;
        ++it_breaks1;
        ++it_spline;
    }
    return this->splineValue(*it_spline, f_value);
}

/**
 * @brief Convert the given value, by using the given polynom.
 * 
 * @param f_coeff The coeffiences of the polynom
 * @param f_value The input value.
 */
template <uint8_t NrBreak, uint8_t NOrd>
float CConverterSpline<NrBreak, NOrd>::splineValue( CCoeffContainerType f_coeff,float f_value){
    float l_res = f_coeff[NOrd];
    for (uint8_t i = 0;i<NOrd;++i)
    {
        l_res+=std::pow(f_value,NOrd-i)*f_coeff[i];
    }
    return l_res;
}

#endif 