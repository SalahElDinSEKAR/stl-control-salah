function set_visible(list_all, list_select)

    for i = 1:numel(list_all)
       set(list_all(i), 'visible', 'off',  'HandleVisibility','off');
    end

    for i = list_select
        set(list_all(i), 'visible', 'on',   'HandleVisibility','on');
    end

end