.. zephyr:code-sample:: ti_eqep_frequency
   :name: TI EQEP frequency measurement
   :relevant-api: counter_interface ti_am3352_eqep_interface

   Measure the frequency of a pulse train with the TI EQEP unit timer.

Overview
********

This sample measures the frequency of a signal using the Frequency Register
(FR) method of the TI Enhanced Quadrature Encoder Pulse (EQEP) peripheral:

- A GPIO driven from a periodic kernel timer generates a 100 Hz, 25% duty
  cycle test signal.
- The EQEP position counter runs in up-count mode and counts one step per
  rising edge of QEPA.
- The EQEP unit timer is armed through the counter alarm API for a 1 second
  window. On each unit timeout the position counter is latched into
  ``QPOSLAT`` and the alarm callback computes the number of edges seen
  during the window, which is the signal frequency in Hz.

The position counter top value is deliberately small so that the counter
wraps every few windows. The sample handles the wraparound when computing
the per-window delta.

Requirements
************

A board with an EQEP instance and a free GPIO, with the GPIO output wired to
the EQEP QEPA input.

On :zephyr:board:`lp_am13e230`, the test signal is driven on PB3 and EQEP0
QEPA is fed from PB11 through the Input X-BAR. Connect BoosterPack header
J4 pin 40 (PB3) to J2 pin 11 (PB11).

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/counter/ti_eqep_frequency
   :board: lp_am13e230
   :goals: build flash
   :compact:

Sample Output
=============

.. code-block:: console

   TI EQEP frequency measurement sample
   Signal: 100 Hz, 25% duty; eQEP clock 200000000 Hz; position top 999
   position  200, measured 100 Hz
   position  300, measured 100 Hz
   ...
   position  900, measured 100 Hz
   position    0, measured 100 Hz (counter wrapped)

Without the jumper the sample reports 0 Hz.
