# HW3 Report 蕭詠倫 109061634

## 用 systemC 程式對 bmp 圖檔進行 Gaussian Blur (高斯模糊)

### 結果

#### 原始圖檔 
![image](https://github.com/lplp9312/ee6470_hw4/blob/master/gau_tlm_bus/lena_std_short.bmp)

#### 模糊處理 
![image](https://github.com/lplp9312/ee6470_hw4/blob/master/gau_tlm_bus/build/out.bmp)

#### 傳送PIXEL數和模擬時間
![image](https://github.com/lplp9312/ee6470_hw4/blob/master/sim_time.jpg)

### systemC 架構
![image](https://github.com/lplp9312/ee6470_hw4/blob/master/arc.jpg)

### systemC程式碼重點介紹

#### 用來進行 Gaussian Blur 的3X3矩陣 (filter_def.h)

        const double mask[MASK_X][MASK_Y] = {0.077847, 0.123317, 0.077847, 0.123317, 0.195346, 0.123317, 0.077847, 0.123317, 0.077847};

#### Testbench,GauFilter,SimpleBus 的建立和 Testbench::i_skt->SimpleBus::t_skt[0]、SimpleBus::i_skt[0]->GauFilter::t_skt 的連接 (main.cpp)

        Testbench tb("tb");
        SimpleBus<1, 1> bus("bus");
        bus.set_clock_period(sc_time(CLOCK_PERIOD, SC_NS));
        GauFilter gau_filter("gau_filter");
        tb.initiator.i_skt(bus.t_skt[0]);
        bus.setDecode(0, Gau_MM_BASE, Gau_MM_BASE + Gau_MM_SIZE - 1);
        bus.i_skt[0](gau_filter.t_skt);


#### Initiator <sc_module> 和 i_skt 的建立，Testbench <sc_module> 的 sub-module (Initiator.h)

        class Initiator : public sc_module
        {
        public:
            tlm_utils::simple_initiator_socket<Initiator> i_skt;

            SC_HAS_PROCESS(Initiator);
            Initiator(sc_module_name n);

            int read_from_socket(unsigned long int addr, unsigned char mask[], unsigned char rdata[], int dataLen);

            int write_to_socket(unsigned long int addr, unsigned char mask[], unsigned char wdata[], int dataLen);

            void do_trans(tlm::tlm_generic_payload &trans);
            tlm::tlm_generic_payload trans;
        };

#### 定義 SimpleBus <sc_module> Debug 會使用到的 print() function (tlm_log.h)

        namespace tshsu {
        std::string print(const unsigned int u);
        std::string print(const sc_dt::uint64 u64);
        std::string print(const tlm::tlm_command command);
        std::string print(const tlm::tlm_sync_enum sync_enum);
        std::string print(const sc_core::sc_time &t, bool unit = true);
        } 

#### SimpleBus <sc_module> 的建立，i_skt,t_stk,MemoryMap 的管理 (SimploBus.h)

        SC_HAS_PROCESS(SimpleBus);
        SimpleBus(sc_core::sc_module_name name, double clock_period_in_ps = 1000,
                    bool trace = false, bool masked = true)
            : sc_core::sc_module(name), MemoryMap(name, NR_OF_INITIATOR_SOCKETS),
                clock_period(clock_period_in_ps, sc_core::SC_PS), m_trace(trace),
                m_is_address_masked(masked) {
            for (unsigned int i = 0; i < NR_OF_TARGET_SOCKETS; ++i) {
                t_skt[i].register_b_transport(this, &SimpleBus::initiatorBTransport, i);
                t_skt[i].register_transport_dbg(this, &SimpleBus::transportDebug, i);
                t_skt[i].register_get_direct_mem_ptr(this, &SimpleBus::getDMIPointer, i);
            }
            for (unsigned int i = 0; i < NR_OF_INITIATOR_SOCKETS; ++i) {
                i_skt[i].register_invalidate_direct_mem_ptr(this, &SimpleBus::invalidateDMIPointers, i);
            }
        }    

#### GauFilter <sc_module> 內部使用的 7條 FIFO_channel 和 t_skt 的建立 (GauFilter.h)

        tlm_utils::simple_target_socket<GauFilter> t_skt;

        sc_fifo<unsigned char> i_r;
        sc_fifo<unsigned char> i_g;
        sc_fifo<unsigned char> i_b;

        sc_fifo<unsigned char> o_result_r;
        sc_fifo<unsigned char> o_result_g;
        sc_fifo<unsigned char> o_result_b;

        sc_fifo<unsigned int> width;

#### blocking_transport 的建立，用來管理 i_skt,t_skt 之間和 t_skt,fifo 之間的溝通 (GauFilter.cpp)

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
            case tlm::TLM_READ_COMMAND:                 //執行 TLM_READ_COMMAND
                switch (addr)
                {
                    case GAU_FILTER_RESULT_ADDR:
                    buffer.uc[0] = o_result_r.read();   //把 RGB data 寫進 buffer
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
                    data_ptr[i] = buffer.uc[i];         //把 buffer data 寫進 payload

                break;

            case tlm::TLM_WRITE_COMMAND:                //執行 TLM_WRITE_COMMAND (第一次傳送 width data，之後皆為 RGB data)
                switch (addr)
                {
                case GAU_FILTER_R_ADDR:
                    if (flag == 0)
                    {
                        for (int i = 0; i < 4; i++)     //把 payload data 寫進 buffer
                            buffer.uc[i] = data_ptr[i];
                        width.write(buffer.uint);       //把 buffer data 寫進 fifo (圖檔的 width data)
                        flag = 1;
                    }
                    else
                    {
                        if (mask_ptr[0] == 0xff)        //把 payload data 寫進 fifo (圖檔的 RGB data)
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

#### Testbench <sc_module> 中，讀取原始圖檔 width,RGB 寫入 socket，並記錄寫入的 pixel 數量 (Testbench.cpp)

        void Testbench::do_gau_blur()
        {
            unsigned char R, G, B;
            int total_pixel = 0;

            word data;
            unsigned char mask[4];

            for (int i = 0; i < 4; i++)
                mask[i] = 0xff;

            data.uint = width; //transfer width information for first time (unsigned int)
            initiator.write_to_socket(Gau_MM_BASE + GAU_FILTER_R_ADDR, mask, data.uc, 4);

            wait(5 * CLOCK_PERIOD, SC_NS);

            for (int i = 0; i < height; i++)
                for (int j = 0; j < width; j++)
                {
                    R = *(source_bitmap + bytes_per_pixel * (width * i + j) + 2);
                    G = *(source_bitmap + bytes_per_pixel * (width * i + j) + 1);
                    B = *(source_bitmap + bytes_per_pixel * (width * i + j) + 0);
                    data.uc[0] = R;
                    data.uc[1] = G;
                    data.uc[2] = B;
                    data.uc[3] = 0;
                    mask[0] = 0xff;
                    mask[1] = 0xff;
                    mask[2] = 0xff;
                    mask[3] = 0;
                    initiator.write_to_socket(Gau_MM_BASE + GAU_FILTER_R_ADDR, mask, data.uc, 4);
                    total_pixel = total_pixel + 1;
                }

            std::cout << std::endl;
            std::cout << "The number of pixels transfer from Testbench to GauFilter = " << total_pixel << std::endl;
            sc_stop();
        }


#### GauFilter <sc_module> 中，從 width,i_r,i_g,i_b <sc_fifo> 讀取資料，並考慮到 data reused，把 3 個 Row 的 data 儲放在 array_r,array_g,array_b，並使用 3X3 Gaussian Blur 矩陣對其進行卷積，並分別寫入 o_result_r,o_result_g,o_result_b <sc_fifo> (GauFilter.cpp)

        void GauFilter::do_filter()
        {
            {
                wait(CLOCK_PERIOD, SC_NS);
            }

            width_in = (int)width.read();       //讀取 width data

            int array_r[MASK_X][width_in + MASK_Y - 1];
            int array_g[MASK_X][width_in + MASK_Y - 1];
            int array_b[MASK_X][width_in + MASK_Y - 1];

            for (int i = 0; i < MASK_X; i++)    //初始化
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
                if (counter == 0)               //第一次讀取 3 個 Row 的 data 
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
                else                            //之後平移 2 個 Row 的 data，並讀取 1 個 Row 的 data
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

                for (int i = 0; i < width_in; i++)      //進行卷積
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

#### Testbench <sc_module> 中，從 socket 讀取資料，並寫入 *target_bitmap 矩陣，作為輸出圖檔的矩陣 (Testbench.cpp)

        void Testbench::do_gau_blur_out()
        {
            word data;
            unsigned char mask[4];

            wait(5 * CLOCK_PERIOD, SC_NS);
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    initiator.read_from_socket(Gau_MM_BASE + GAU_FILTER_RESULT_ADDR, mask, data.uc, 4);
                    *(target_bitmap + bytes_per_pixel * (width * i + j) + 2) = data.uc[0];
                    *(target_bitmap + bytes_per_pixel * (width * i + j) + 1) = data.uc[1];
                    *(target_bitmap + bytes_per_pixel * (width * i + j) + 0) = data.uc[2];
                }
            }
            sc_stop();
        }


