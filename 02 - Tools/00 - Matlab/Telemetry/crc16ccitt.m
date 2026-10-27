function crc = crc16ccitt(data)
%CRC16CCITT  CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), as tlm_crc16() in C.
%   Check: crc16ccitt(uint8('123456789')) == hex2dec('29B1')
persistent T
if isempty(T)
    T = zeros(1,256,'uint16');
    for b = 0:255
        c = bitshift(uint16(b), 8);
        for k = 1:8
            if bitand(c, uint16(32768))
                c = bitxor(bitshift(c,1), uint16(4129));   % bitshift truncates overflowing bits
            else
                c = bitshift(c,1);
            end
        end
        T(b+1) = c;
    end
end
crc = uint16(65535);
for x = uint16(data(:)).'
    idx = bitxor(bitshift(crc,-8), x);
    crc = bitxor(bitshift(crc,8), T(double(idx)+1));
end
crc = double(crc);
end
