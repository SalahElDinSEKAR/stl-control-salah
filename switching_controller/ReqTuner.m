classdef ReqTuner < BreachGuiEditParams

    methods
        function this=  ReqTuner(B,phi)
            
            if nargin ==0
                phi = 'alw (x1[t] < x_threshold) and ev_[0, t_lim] (x2[t] > y_truc)';
            end
            
            R = BreachRequirement(phi);
            
            this = this@BreachGuiEditParams(R);
            this.data_gui.BrSet = R;
            this.data_gui.B = B;  % on est pas rendu.
            
            this.data_gui.req_fig = [];

            callback_button_eval = @(o,e)(this.callback_eval_button());
            this.create_button('button_eval', ['Eval ' disp(R.req_monitors{1}.formula)], callback_button_eval)

            layout = [this.layout ; {{ 'button_eval' }  }];
            set(this.hdle,'Name', ['ReqTuner ' disp(R.req_monitors{1}.formula)] )
            this.set_layout(layout);
            this.enable_resizable();
        end


        function callback_eval_button(this)
            pos = get(this.hdle,'Position');                
            if ~isempty(this.data_gui.req_fig)
                try 
                    pos_plot = get(this.data_gui.req_fig.Fig, 'Position');
                    delete(this.data_gui.req_fig.Fig)
                    this.data_gui.req_fig = [];
                catch
                    warning('Figure was closed or deleted.')
                end                
            end
            
            R = this.data_gui.BrSet.copy();            
            B  = this.data_gui.B;
            R.Eval(B);
            this.data_gui.req_fig = BreachSamplesPlot(R);
            this.data_gui.req_fig.set_y_axis('sum');
            if ~exist('pos_plot', 'var')            
                pos_plot = get(this.data_gui.req_fig.Fig, 'Position');
                pos_plot(1) = pos(1)+pos(3);
                pos_plot(2) = pos(2);
            end
            set(this.data_gui.req_fig.Fig, 'Position', pos_plot);
        end

    end


end