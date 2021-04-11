#include <cmath>
#include <iomanip>

#include "GauFilter.h"

GauFilter::GauFilter(sc_module_name n)
    : sc_module(n), t_skt("t_skt"), base_offset(0)
{
  SC_THREAD(do_filter);

  t_skt.register_b_transport(this, &GauFilter::blocking_transport);
}

//const double mask[MASK_X][MASK_Y] = {0.077847, 0.123317, 0.077847, 0.123317, 0.195346, 0.123317, 0.077847, 0.123317, 0.077847};

void GauFilter::do_filter()
{
  {
    wait(CLOCK_PERIOD, SC_NS);
  }

  width_in = (int)width.read();

  int array_r[MASK_X][width_in + MASK_Y - 1];
  int array_g[MASK_X][width_in + MASK_Y - 1];
  int array_b[MASK_X][width_in + MASK_Y - 1];

  for (int i = 0; i < MASK_X; i++)
  {
    for (int j = 0; j < width_in + MASK_Y - 1; j++)
    {
      array_r[i][j] = 0;
      array_g[i][j] = 0;
      array_b[i][j] = 0;
    }
  }

  int counter = 0;

  while (true)
  {
    if (counter == 0)
    {
      for (int i = 0; i < MASK_X; i++)
      {
        for (int j = 0; j < width_in; j++)
        {
          array_r[i][j] = i_r.read();
          array_g[i][j] = i_g.read();
          array_b[i][j] = i_b.read();
          wait(CLOCK_PERIOD, SC_NS);
        }
      }
      counter = MASK_X;
    }
    else
    {
      for (int i = 0; i < MASK_X - 1; i++)
      {
        for (int j = 0; j < width_in; j++)
        {
          array_r[i][j] = array_r[i + 1][j];
          array_g[i][j] = array_g[i + 1][j];
          array_b[i][j] = array_b[i + 1][j];
        }
      }
      for (int k = 0; k < width_in; k++)
      {
        array_r[MASK_X - 1][k] = i_r.read();
        array_g[MASK_X - 1][k] = i_g.read();
        array_b[MASK_X - 1][k] = i_b.read();
        wait(CLOCK_PERIOD, SC_NS);
      }
      counter++;
    }

    for (int i = 0; i < width_in; i++)
    {
      val_r = 0;
      val_g = 0;
      val_b = 0;
      wait(CLOCK_PERIOD, SC_NS);
      for (int j = 0; j < MASK_Y; j++)
      {
        for (int k = 0; k < MASK_X; k++)
        {
          val_r += array_r[k][j + i] * mask[k][j];
          val_g += array_g[k][j + i] * mask[k][j];
          val_b += array_b[k][j + i] * mask[k][j];
          wait(CLOCK_PERIOD, SC_NS);
        }
      }
      o_result_r.write(val_r);
      o_result_g.write(val_g);
      o_result_b.write(val_b);
    }
  }
}

void GauFilter::blocking_transport(tlm::tlm_generic_payload &payload, sc_core::sc_time &delay)
{
  sc_dt::uint64 addr = payload.get_address();
  addr = addr - base_offset;
  unsigned char *mask_ptr = payload.get_byte_enable_ptr();
  unsigned char *data_ptr = payload.get_data_ptr();

  word buffer;

  for (int i = 0; i < 4; i++)
    buffer.uc[i] = 0;

  switch (payload.get_command())
  {
  case tlm::TLM_READ_COMMAND:
    switch (addr)
    {
    case GAU_FILTER_RESULT_ADDR:
      buffer.uc[0] = o_result_r.read();
      buffer.uc[1] = o_result_g.read();
      buffer.uc[2] = o_result_b.read();
      break;
    default:
      std::cerr << "Error! GauFilter::blocking_transport: address 0x"
                << std::setfill('0') << std::setw(8) << std::hex << addr
                << std::dec << " is not valid" << std::endl;
      break;
    }
    for (int i = 0; i < 4; i++)
      data_ptr[i] = buffer.uc[i];

    break;

  case tlm::TLM_WRITE_COMMAND:
    switch (addr)
    {
    case GAU_FILTER_R_ADDR:
      if (flag == 0)
      {
        for (int i = 0; i < 4; i++)
          buffer.uc[i] = data_ptr[i];
        width.write(buffer.uint);
        flag = 1;
      }
      else
      {
        if (mask_ptr[0] == 0xff)
        {
          i_r.write(data_ptr[0]);
        }
        if (mask_ptr[1] == 0xff)
        {
          i_g.write(data_ptr[1]);
        }
        if (mask_ptr[2] == 0xff)
        {
          i_b.write(data_ptr[2]);
        }
      }
      break;
    default:
      std::cerr << "Error! GauFilter::blocking_transport: address 0x"
                << std::setfill('0') << std::setw(8) << std::hex << addr
                << std::dec << " is not valid" << std::endl;
      break;
    }
    break;

  case tlm::TLM_IGNORE_COMMAND:
    payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
    return;
  default:
    payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
    return;
  }

  payload.set_response_status(tlm::TLM_OK_RESPONSE);
}
