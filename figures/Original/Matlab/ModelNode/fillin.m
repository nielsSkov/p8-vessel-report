function fillin_data = fillin(time_base,time_data,data)
%function fillin_data = fillin(time_base,time_data,data)

% Get vector lengths
nbase = length(time_base);
ndata = length(time_data);

% Iitialize data
fillin_data = zeros(1,nbase);

% Fill in the data
k = 1;
for i=2:1:ndata
    while ((time_base(k) < time_data(i)) && (k < nbase))
        fillin_data(k) = data(i-1);
        k = k + 1;
    end  
end

end