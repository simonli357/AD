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

#ifndef QUEUE_TPP
#define QUEUE_TPP

#ifndef QUEUE_HPP
#error __FILE__ should only be included from queue.hpp.
#endif



/** @brief  Queue Class constructor
 *
 *  Constructor method
 *
 *  
 */
template <class T, unsigned int N>
CQueue<T,N>::CQueue()
    : m_buffer()
    , m_start(0)
    , m_end(N-1)
    , m_size(0)
{
}

/** @brief  Queue Class destructor
 *
 *  Destructor method
 *
 *  
 */
template <class T, unsigned int N>
CQueue<T,N>::~CQueue()
{
}

/** @brief  Is full method
 *
 *  
 *  @return    True if Queue FULL
 */
template <class T, unsigned int N>
bool CQueue<T,N>::isFull()
{
    return (m_start + 1)%N == m_end;
}

/** @brief  Is empty method
 *
 *  
 *  @return    True if Queue EMPTY
 */
template <class T, unsigned int N>
bool CQueue<T,N>::isEmpty()
{
    return (m_end + 1)%N == m_start;
}

/** @brief  Peek method
 *
 *  
 *  @return    Value on top of queue
 */
template <class T, unsigned int N>
T CQueue<T,N>::peek()
{
    return m_buffer[(m_end+1) % N];
}

/** @brief  Pop method
 * 
 *  Method for removing the item on top of the queue and returning it
 *
 *  
 *  @return    Element at top of the queue
 */
template <class T, unsigned int N>
T CQueue<T,N>::pop()
{
    if(!isEmpty())
    {
        m_end = (m_end+1) % N;
        T l_char = m_buffer[m_end];
        m_size--;
        return l_char;
    }
    else
    {
        return 0;
    }
}

/** @brief  Push method
 * 
 *  Method for inserting an item on top of the queue
 *
 *  @param f_char    element to be added to the queue
 *  @return    None
 */
template <class T, unsigned int N>
void CQueue<T,N>::push(T& f_char)
{
    if (!isFull())
    {
        m_buffer[m_start] = f_char;
        m_start = (m_start+1) % N; 
        m_size++;           
    }
}

/** @brief  Push method
 * 
 *  Method for inserting a sequence of items on top of the queue
 *
 *  @param f_char    pointer to element to be added to the queue
 *  @param f_len     number of elements to be added
 *  @return    None
 */
template <class T, unsigned int N>
inline void CQueue<T,N>::push(T *f_char, unsigned int f_len)
{
    for ( unsigned int l_idx = 0; l_idx < f_len; ++l_idx)
    {
        push(f_char[l_idx]);
    }
}

/** @brief  Get size method
 * 
 *  Returns the size of the queue
 *
 *  
 *  @return    Queue size
 */
template <class T, unsigned int N>
unsigned int CQueue<T,N>::getSize()
{
    return m_size;
}

/** @brief  Empty queue method
 *
 *  
 *  @return    None
 */
template <class T, unsigned int N>
inline void CQueue<T,N>::empty()
{
    m_start = 0;
    m_end = N-1;
    m_size = 0;
}

#endif // QUEUE_TPP