function insert_zero_divide_avoidance(file_path)
LIKE = [char(39), 'like', char(39)];
DIV_MIN = 'div_min';

%% ファイルの文字列を読む
target_fID = fopen(file_path);
text = textscan(target_fID, '%s', 'delimiter', '\n', 'whitespace', '');
full_text = text{1,1};
new_text = '';
[file_dir, file_name] = get_file_info(file_path);

%% 関数名を変える
full_text{1, 1} = strrep(full_text{1, 1}, file_name, [file_name, '_azd']);

%% 関数の引数部分にDIV_MINを加える
full_text{1, 1} = strrep(full_text{1, 1}, ')', [',', DIV_MIN, ')']);

%% 割り算「./」が使われている箇所を探す
div_position = cell(size(full_text));

for i = 1:size(div_position, 1)
    div_position{i} = strfind(full_text{i, 1}, "./");
end

for i = 1:size(div_position, 1)
    
    if isempty(div_position{i, 1})
        new_text = [new_text, full_text{i, 1}, newline];
        continue;
    end
    
    num_of_div = size(div_position{i, 1}, 2);
    splitted_text = cell(1, num_of_div + 1);
    replaced_splitted_text = cell(1, num_of_div + 1);
    
    %% 関数を挿入するために、文字列を区切る
    split_position = div_position{i, 1} + 1;
    
    splitted_text{1, 1} = full_text{i, 1}(1:split_position(1, 1));
    
    if (num_of_div == 1)
        splitted_text{1, 2} = full_text{i, 1}(split_position(1, 1) + 1:end);
    else
        for j = 2:num_of_div
            splitted_text{1, j} = ...
                full_text{i, 1}(split_position(1, j - 1) + 1:split_position(1, j));
        end
        splitted_text{1, end} = ...
                full_text{i, 1}(split_position(1, end) + 1:end);
    end
    
    replaced_splitted_text{1, 1} = splitted_text{1, 1};
    
    %% 割る変数の部分を抽出する
    for j = 1:num_of_div
        semicolon_pos   = find_div_point(splitted_text{1, j + 1}, ";");
        comma_pos       = find_div_point(splitted_text{1, j + 1}, ",");
        plus_pos        = find_div_point(splitted_text{1, j + 1}, "+");
        minus_pos       = find_div_point(splitted_text{1, j + 1}, "-");
        mul_pos         = find_div_point(splitted_text{1, j + 1}, ".*");
        sq_bracket_pos  = find_div_point(splitted_text{1, j + 1}, "]");
        paren_start_pos = find_div_point(splitted_text{1, j + 1}, "(");
        
        [first_pos, pos_type] = min([semicolon_pos(1), comma_pos(1), ...
            plus_pos(1), minus_pos(1), mul_pos(1), sq_bracket_pos(1), ...
            paren_start_pos(1)]);
        
        if (pos_type < 7)
            div_num_text = splitted_text{1, j + 1}(1:first_pos - 1);
        else
            % かっこが来た時の特別処理
            text = splitted_text{1, j + 1};
            paren_b_exist = 1;
            for k = 1:numel(text)
                if (text(k) == ')')
                    paren_b_exist = paren_b_exist - 1;
                    
                    if (paren_b_exist == 1)
                        break;
                    end
                end
                
                if (text(k) == '(')
                    paren_b_exist = paren_b_exist + 1;
                end
            
            end
            
            first_pos = k + 1;
            div_num_text = splitted_text{1, j + 1}(1:k);
        end
        
        rep_str = add_avoidance_function(div_num_text, file_name, DIV_MIN);
        replaced_splitted_text{1, j + 1} = [rep_str, splitted_text{1, j + 1}(first_pos:end)];
    
    end
    
    for j = 1:size(replaced_splitted_text, 2)
        new_text = [new_text, replaced_splitted_text{1, j}];
    end
    new_text = [new_text, newline];

end

%% ファイルの末尾に「avoid_zero_divide_~~」関数を定義する
azd_text = [ newline, ...
    'function out = ', 'avoid_zero_divide_', file_name, '(in, ', DIV_MIN, ')', newline, ...
    'if (in >= 0)', newline, ...
    '    if (in < div_min)', newline, ...
    '        out = cast(div_min, ', LIKE, ', ', 'in);', newline, ...
    '    else', newline, ...
    '        out = in;', newline, ...
    '    end', newline, ...
    'else', newline, ...
    '    if (in > -div_min)', newline, ...
    '        out = cast(-div_min, ', LIKE, ', ', 'in);', newline, ...
    '    else', newline, ...
    '        out = in;', newline, ...
    '    end', newline, ...
    'end', newline, ...
    newline, ...
    'end', newline];

fclose(target_fID);

%% ゼロ割回避を施した関数を別ファイルに出力する
fileID = fopen([file_dir, filesep, file_name, '_azd.m'], 'wt');
fprintf(fileID,'%s',[new_text, azd_text]);
fclose(fileID);

end

function pos = find_div_point(text, sep)

pos = strfind(text, sep);
% sepが見つからない場合は、minでカウントしないようにNaNにする
if isempty(pos)
    pos = NaN;
end

end

function [file_dir, file_name] = get_file_info(file_path)
% file_pathからファイル名、ディレクトリを抽出する

name_list = strsplit(file_path, filesep);
file_dir = strrep(file_path, [filesep, name_list{1, end}], '');

% 拡張子を消す
name = strsplit(name_list{1, end}, '.');
file_name = name{1, 1};

end

function text = add_avoidance_function(div_num_text, file_name, DIV_MIN)
% 汎用性を持たせるため、ゼロ割回避関数にファイル名を付けて文字列を生成する

text = ['avoid_zero_divide_', file_name, '(', div_num_text, ', ', ...
        DIV_MIN, ')'];

end

