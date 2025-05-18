#ifndef FILTER_TPP
#define FILTER_TPP

#ifndef FILTER_HPP
#error __FILE__ should only be included from filter.hpp.
#endif // FILTER_HPP

template <class T, uint32_t NC>
using CMeasurementType = utils::linalg::CColVector<T,NC>;

/******************************************************************************/
/** @brief  CIIRFilter Class constructor
 *
 *
 *  @param f_A   the feedback filter coefficients 
 *  @param f_B   the feedforward filter coefficients
 */
template <class T, uint32_t NA, uint32_t NB>
signal::filter::lti::siso::CIIRFilter<T,NA,NB>::CIIRFilter(const utils::linalg::CRowVector<T,NA>& f_A,const utils::linalg::CRowVector<T,NB>& f_B) 
    : m_A(f_A)
    , m_B(f_B)
    , m_Y()
    , m_U() 
{
}

/** @brief  Operator to apply the filtering
  *
  * @param f_u                 the input data
  * @return                    the filtered output data
  */
template <class T, uint32_t NA, uint32_t NB>
T signal::filter::lti::siso::CIIRFilter<T,NA,NB>::operator()(T& f_u)
{
    for (uint32_t l_idx = NB-1; l_idx > 0; --l_idx)
    {
        m_U[l_idx] = m_U[l_idx-1];
    }
    m_U[0][0] = f_u;

    utils::linalg::CMatrix<T,1,1> l_y = m_B*m_U - m_A*m_Y;
    // T l_y = m_B*m_U - m_A*m_Y;

    for (uint32_t l_idx = NA-1; l_idx > 0 ; --l_idx)
    {
        m_Y[l_idx] = m_Y[l_idx-1];
    }
    m_Y[0][0] = l_y[0][0];

    return m_Y[0][0];
}

/******************************************************************************/
/** @brief  CFIRFilter Class constructor
 *
 *
 *  @param f_B   the feedforward filter coefficients
 */
template <class T, uint32_t NB>
signal::filter::lti::siso::CFIRFilter<T,NB>::CFIRFilter(const utils::linalg::CRowVector<T,NB>& f_B) 
    : m_B(f_B), m_U() 
{
}

/** @brief  Operator to apply the filtering
  *
  * @param f_u                 the input data
  * @return                    the filtered output data
  */
template <class T, uint32_t NB>
T signal::filter::lti::siso::CFIRFilter<T,NB>::operator()(T& f_u)
{
    for (uint32_t l_idx = NB-1; l_idx > 0; --l_idx)
    {
        m_U[l_idx] = m_U[l_idx-1];
    }
    m_U[0] = f_u;

    utils::linalg::CMatrix<T,1,1> l_y = m_B*m_U;

    return l_y[0][0];
}

/******************************************************************************/
/** @brief  MeanFilter Class constructor
 *
 *  
 */
template <class T, uint32_t NB> 
signal::filter::lti::siso::CMeanFilter<T,NB>::CMeanFilter() 
    : m_B(1./NB)
    , m_U() 
{
}

/** @brief  Operator to apply the filtering
  *
  * @param f_u                 the input data
  * @return                     the filtered output data 
  */
template <class T, uint32_t NB> 
T signal::filter::lti::siso::CMeanFilter<T,NB>::operator()(T& f_u)
{
    T l_y =0;

    for (uint32_t l_idx = 1; l_idx < NB; ++l_idx)
    {
        m_U[l_idx][0] = m_U[l_idx-1][0];
        l_y += m_U[l_idx][0];
    }
    m_U[0][0] = f_u;
    l_y += m_U[0][0];

    return m_B*l_y;
}



/******************************************************************************/
/** @brief  CMedianFilter Class constructor
 *
 *  Constructor method
 *
 *  
 */
template <class T, uint32_t N>
signal::filter::nlti::siso::CMedianFilter<T,N>::CMedianFilter()
    :m_median()
    ,m_smallest()
    ,m_new(0)
    ,m_size(N)
    ,m_queue()
{

    for (unsigned int l_idx = 0; l_idx < N ; l_idx++)
    {
        m_queue[l_idx].info = 0;
        m_queue[l_idx].next = &(m_queue[(l_idx + 1) % N]);
        m_queue[l_idx].prev = &(m_queue[(N + l_idx - 1) % N]);
    }

    m_new = N - 1;
    m_size = N;
    m_median = &(m_queue[m_size / 2]);
    m_smallest =&(m_queue[0]);
}

/** @brief  Filtering the values. 
  *
  * @param f_val            the input data
  * @return                 filtered the data 
  */
template <class T, uint32_t N>
T signal::filter::nlti::siso::CMedianFilter<T,N>::operator()(T& f_val)
{
    m_new = (m_new + 1) % m_size;		 //shift new index	 //inca e valoarea veche
    m_queue[m_new ].info = f_val;		 //suprascrie cea mai veche valoare

    //ordonare dupa valoare

        //elementul new se "scoate" din lista
    m_queue[m_new].prev->next = m_queue[m_new].next;			 //5.
    m_queue[m_new].next->prev = m_queue[m_new].prev;			 //6.

        //update smallest value
    my_structure* l_i = m_smallest;
    if (&(m_queue[m_new]) == m_smallest)
    {
        if (m_queue[m_new].info > m_smallest->next->info)
        {
            m_smallest = m_smallest->next;
            l_i = m_smallest;
        }
    }
    else if (m_queue[m_new].info <= m_smallest->info)
    {
        l_i = m_smallest;
        m_smallest = &m_queue[m_new];
    }

        //cautarea locului unde trebuie sa se amplaseze noul element in lista
    unsigned int l_cnt = 1;
    if (&(m_queue[m_new]) == l_i)
    {
        l_i = l_i->next;
    }

    while ((l_i->info < m_queue[m_new].info) && (l_cnt <= m_size - 1))
    {
        l_i = l_i->next;
        l_cnt++;
    }

        //inserarea elemntului new la locul potrivit
    l_i->prev->next = &m_queue[m_new];							 //1.
    m_queue[m_new].next = l_i;									 //2.
    m_queue[m_new].prev = l_i->prev;							 //3.
    l_i->prev = &m_queue[m_new];									 //4.

    //varianta ineficienta pentru aflarea medianului  cand filtrul are dimensiuni mari
    m_median=m_smallest;
    for(uint8_t iddx=0; iddx < m_size/2; ++iddx)
    {
        m_median=m_median->next;
    }

    return getMedian();
}

/*  */
/** @brief Get medianator
  *
  * 
  * @return    median value
  */
template <class T, uint32_t N>
T signal::filter::nlti::siso::CMedianFilter<T, N>::getMedian()
{
    T ret_val;
    if (1 == m_size % 2)		// daca filtrul are lungime impara
    {
        ret_val = m_median->info;
    }
    else						// daca filtrul are lungime para
    {
        ret_val = 0.5*(m_median->info + m_median->prev->info);
    }
    return ret_val;
}

#endif