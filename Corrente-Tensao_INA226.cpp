/*

O INA226 (Texas Instruments) é um conversor analógico-digital de 16 bits dedicado a medir:

    Grandeza               |	      Faixa típica (com VBUS=36 V)
--------------------------------------------------------------------------
Tensão no barramento VBUS	|            0 – 36 V
Queda no shunt VSHUNT	    |           ±81,92 mV
Corrente I	                |        Definida pelo valor do resistor shunt e calibração
Potência P	                |        Calculada internamente (I × VBUS)

Ele amostra duas tensões (barramento e shunt), converte em corrente e potência e dispõe de alarme programável para sobrecorrente, sobretensão ou sobrepotência. A interface é I²C até 400 kHz.


##########################################################################################

Ligação típica:

INA226	        Conexão	                                Observação
IN+	       Lado “barramento” do shunt	            Próximo ao polo negativo do pack
IN-        Lado “carga” do shunt	                -
SCL/SDA	   GPIO 22 / GPIO 21 (ESP32 padrão I²C)	    Pull-ups 4 k7 a 3 V3
A0/A1	   Endereço I²C (GND/3V3)	                Permite até 4 dispositivos
VCC	       3,0 – 3,6 V	                            Mesmo 3 V3 do ESP32
GND	       GND                                      comum LV



Shunt      IN+ ───\/\/─── IN-  (INA226)
Pack(-) ──┘                └─> para inversor / BMS


*/