function s = decodeState(p)
%DECODESTATE  Payload TYPE 0x01 (71 bytes) -> struct. Layout = tlm_state_t in C.
p = uint8(p);
f = typecast(p(7:70), 'single');
s.seq   = double(typecast(p(1:2), 'uint16'));
s.t_us  = double(typecast(p(3:6), 'uint32'));
s.q     = double(f(1:4)).';        % [w x y z], body -> world
s.bias  = double(f(5:7)).';        % rad/s
s.acc   = double(f(8:10)).';       % g
s.gyro  = double(f(11:13)).';      % rad/s
s.mag   = double(f(14:16)).';      % uT
s.flags = double(p(71));
end
