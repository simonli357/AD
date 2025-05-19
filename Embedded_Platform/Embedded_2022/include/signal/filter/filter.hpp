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

/* Include guard */
#ifndef FILTER_HPP
#define FILTER_HPP

#include <utils/linalg/linalg.h>
#include <signal/systemmodels/systemmodels.hpp>

namespace signal::filter
{
    /**
     * @brief The filter interface
     * 
     * @tparam T The type of the input and output signal
     */
    template<class T>
    class IFilter
    {
        public:
            virtual T operator()(T&)=0;
    }; // Class IFilter

    namespace lti
    {
        namespace siso
        {

            /**
             * @brief Infinite impulse response (IIR) discrete-time filter template class
             * 
             * @tparam T    The type of the input and output signal
             * @tparam NA   Number of coefficients for feedback filter
             * @tparam NB   Number of coefficients for feedforward filter
             */
            template <class T, uint32_t NA, uint32_t NB>
            class CIIRFilter:public IFilter<T>
            {
                public:
                    /* Constructor */
                    CIIRFilter(const utils::linalg::CRowVector<T,NA>& f_A,const utils::linalg::CRowVector<T,NB>& f_B);
                    /* Operator */
                    T operator()(T& f_u);
                private:
                    CIIRFilter(){}
                    /** @brief Polynomial coefficient for feedback filter  */
                    utils::linalg::CRowVector<T,NA> m_A;
                    /** @brief Polynomial coefficient for feedforward filter */
                    utils::linalg::CRowVector<T,NB> m_B;
                    /** @brief Memory for feedback filter  */
                    utils::linalg::CColVector<T,NA> m_Y;
                    /** @brief Memory for feedback filter  */
                    utils::linalg::CColVector<T,NB> m_U;
            }; // class CIIRFilter

            /**
             * @brief Finite impulse response (FIR) discrete-time filter.
             * 
             * @tparam T    The type of the input and output signal
             * @tparam NB   Number of coefficients for feedforward filter
             */
            template <class T, uint32_t NB>
            class CFIRFilter:public IFilter<T>
            {
                public:
                    /* Constructor */
                    CFIRFilter(const utils::linalg::CRowVector<T,NB>& f_B);
                    /* Operator */
                    T operator()(T& f_u);
                private:
                    CFIRFilter() {}
                    /** @brief Polynomial coefficient for feedforward filter */
                    utils::linalg::CRowVector<T,NB> m_B;
                    /** @brief Memory for feedback filter  */
                    utils::linalg::CColVector<T,NB> m_U;
            }; // class CFIRFilter

            /**
             * @brief Mean filter or average filter
             * 
             * @tparam T    The type of the input and output signal
             * @tparam NB   Number of memorized values for calculating mean value
             */
            template <class T, uint32_t NB>
            class CMeanFilter:public IFilter<T>
            {
                public:
                    /* Constructor */
                    CMeanFilter();
                    /* Operator */
                    virtual T operator()(T& f_u);
                private:
                    T m_B;
                    utils::linalg::CColVector<T,NB> m_U;
            }; // class CMeanFilter
        }; // namespace siso
    }; // namespace linear


    namespace nlti
    {
        namespace siso
        {   

            /**
             * @brief  Median filter - It results the middle value after the input values have been sorted. 
             * 
             * @tparam T        type of the values
             * @tparam N        size of the filter
             */
            template <class T, uint32_t N>
            class CMedianFilter:public IFilter<T>
            {
            public:
                /* Constructor */
                CMedianFilter();
                inline T addNewValue(T& f_val);
                inline T getMedian();
                inline T operator()(T& f_v);

            private:
                struct my_structure
                {
                    T info;
                      my_structure *next;
                      my_structure *prev;
                    
                };

                my_structure *m_median;    //pointer to the list element that represents the median
                my_structure *m_smallest;  //pointer to the smallest element of the list (the first)
                uint32_t m_new;		     //index to the newest element in the list
                uint32_t m_size;		     //the size of the list/queue

                my_structure m_queue[N];
            }; // class CMedianFilter
        }; // namespace siso
    }; // namespace nonlinear 
}; // namespace singal::filter

#include "filter.tpp"

#endif // FILTER_H
