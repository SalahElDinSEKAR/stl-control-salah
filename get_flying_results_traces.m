init_flying;
load data/Res_Flying_Success15-03.mat


init_sampling100.num_corners= 16;
init_sampling100.num_quasi_random = 84;
init_sampling100.scaling = .8;
init_sampling100.seed=1;

init_sampling1000.num_corners= 256;
init_sampling1000.num_quasi_random = 744;
init_sampling1000.scaling = .8;
init_sampling1000.seed=1;

init_sampling5000.num_corners= 256;
init_sampling5000.num_quasi_random = 5000;
init_sampling5000.scaling = .8;
init_sampling5000.seed=1;


for inn = 1:numel(all_results)       
    nn = all_results{inn}.nn;
    [~, Bnn100(inn)] = pb_flying.get_nn_cex_traces(nn, init_sampling100);    
end

%%
for inn = 1:numel(all_results)        
    nn = all_results{inn}.nn;
    [~, Bnn1000(inn)] = pb_flying.get_nn_cex_traces(nn, init_sampling1000);    
end

%%
for inn = 1:2:numel(all_results)        
    inn
    nn = all_results{inn}.nn;
    [~, Bnn5000(inn)] = pb_flying.get_nn_cex_traces(nn, init_sampling5000);    
end

