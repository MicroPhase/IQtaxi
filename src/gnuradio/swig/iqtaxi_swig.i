/* -*- c++ -*- */

#define IQTAXI_API

%include "gnuradio.i"

%{
#include "gnuradio/iqtaxi/rx_source.h"
#include "gnuradio/iqtaxi/tx_sink.h"
%}

%include "gnuradio/iqtaxi/rx_source.h"
%include "gnuradio/iqtaxi/tx_sink.h"

GR_SWIG_BLOCK_MAGIC2(iqtaxi, rx_source);
GR_SWIG_BLOCK_MAGIC2(iqtaxi, tx_sink);
