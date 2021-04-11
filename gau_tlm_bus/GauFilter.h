#ifndef GAU_FILTER_H_
#define GAU_FILTER_H_
#include <systemc>
using namespace sc_core;

#include "tlm"
#include "tlm_utils/simple_target_socket.h"

#include "filter_def.h"

class GauFilter : public sc_module
{
public:
  tlm_utils::simple_target_socket<GauFilter> t_skt;

  sc_fifo<unsigned char> i_r;
  sc_fifo<unsigned char> i_g;
  sc_fifo<unsigned char> i_b;

  sc_fifo<unsigned char> o_result_r;
  sc_fifo<unsigned char> o_result_g;
  sc_fifo<unsigned char> o_result_b;

  sc_fifo<unsigned int> width;

  SC_HAS_PROCESS(GauFilter);
  GauFilter(sc_module_name n);
  ~GauFilter() = default;

private:
  void do_filter();

  int val_r;
  int val_g;
  int val_b;

  int flag = 0;

  int width_in;
  
  unsigned int base_offset;
  void blocking_transport(tlm::tlm_generic_payload &payload, sc_core::sc_time &delay);
};
#endif
