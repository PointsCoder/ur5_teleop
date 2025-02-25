sudo chmod 666 /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT94EJQJ-if00-port0
python scripts/gello_get_offset.py \
    --start-joints 0 -1.57 1.57 -1.57 -1.57 0 \
    --joint-signs 1 1 -1 1 1 1 \
    --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT94EJQJ-if00-port0