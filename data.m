function data

fid = fopen('database.txt', 'w+');

for i=1:450
    a=num2str(i);
    c1='.jpg';
    filename=strcat(a,c1);
    fprintf(fid,'%s\r',filename);
end
fclose(fid);