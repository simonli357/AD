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
#ifndef QUEUE_HPP
#define QUEUE_HPP

namespace utils
{
    /**
     * @brief It is used for executing a queue container.
     * 
     * @tparam T The type of the varieble
     * @tparam N The size of the queue
     */
    template <class T, unsigned int N>
    class CQueue
    {
        public:
            /* Constructor */
            CQueue();
            /* Destructor */
            virtual ~CQueue();
            /* Is full method */
            inline bool isFull();
            /* Is full method */
            inline bool isEmpty();
            /* Peek */
            inline T peek();
            /* Pop */
            inline T pop();
            /* Get queue size */
            inline unsigned int getSize();
            /* Push single element */
            inline void push(T& f_char);
            /* Push multiple elements */
            inline void push(T *f_char, unsigned int f_len);
            /* Empty queue */ 
            inline void empty();
        private:
            /* buffer */
            volatile T m_buffer[N];
            /* start */
            volatile unsigned int  m_start;
            /* end */
            volatile unsigned int  m_end;
            /* size */
            volatile unsigned int  m_size;
    }; // class CQueue
    #include "queue.tpp"
}; // namespace utils

#endif // QUEUE_HPP
