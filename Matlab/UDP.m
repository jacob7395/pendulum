IP = '192.168.168.2';
obj1 = instrfind('Type', 'udp', 'RemoteHost', IP, 'RemotePort', 3333, 'Tag', '');

% Create the udp object if it does not exist
% otherwise use the object that was found.
if isempty(obj1)
    obj1 = udp(IP, 3333);
else
    fclose(obj1);
    obj1 = obj1(1)
end

% Configure instrument object
% these our our ip and port
% port must be > 1024
set(obj1, 'LocalHost', IP);
set(obj1, 'LocalPort', 63205);
set(obj1, 'LocalPortMode', 'manual');

% Connect to instrument object
fopen(obj1);

while ~isequal(count,loop)

    % wait for udp packet
    % default time = 10 sec but can be changed
    s = fscanf(obj1);

    % breakpoint here always works but when runniing without
    % then s is always the last value ????
    voltage(count)=str2double(s);

end

% Disconnect from instrument object, obj1.
fclose(obj1);

% Clean up all objects.
delete(obj1);