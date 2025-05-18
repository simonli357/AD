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

#ifndef CONVERTERS_H
#define CONVERTERS_H

#include<cmath>
#include<stdint.h>
#include<array>

namespace signal{
  namespace controllers{
      /**
       * @brief Converter interface with single input and single output
       * 
       */
      class IConverter{
        public:
          virtual float operator()(float) = 0;
      };

      /**
       * @brief Polynomial converter.
       * 
       * @tparam NOrd The degree of the polynomial converter
       */
      template<uint8_t NOrd>
      class CConverterPolynom:public IConverter
      {
        public:
          CConverterPolynom(std::array<float,NOrd+1>);
          float operator()(float);

        private:
          std::array<float,NOrd+1> m_coeff;
      };

      /**
       * @brief A converter based on the set of break point and the multiple polynomial function.
       * 
       * @tparam NrBreak Number of the break.
       * @tparam NOrd    Degree of the polynomial converter. 
       */
      template<uint8_t NrBreak,uint8_t NOrd>
      class CConverterSpline:public IConverter
      {
        public:
          /** @brief Coefficient container types */
          using CCoeffContainerType = std::array<float,NOrd+1>;
          /** @brief Splines container type */
          using CSplineContainerType = std::array<CCoeffContainerType,NrBreak+1>;
          /** @brief Breaks container type */
          using CBreakContainerType = std::array<float,NrBreak>;

          CConverterSpline(CBreakContainerType f_breaks,CSplineContainerType f_splines);
          float operator()(float);
        private:
          float splineValue( CCoeffContainerType,float);

          CBreakContainerType     m_breaks;
          CSplineContainerType    m_splines;
          
      };

      #include "converters.tpp"
  }; //namespace controllers

}; // namespace signal


#endif // CONVERTERS_H
