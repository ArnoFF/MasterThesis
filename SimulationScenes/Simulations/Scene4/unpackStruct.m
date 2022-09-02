function unpackStruct(structure)
    fn = fieldnames(structure);
    for i = 1:numel(fn)
        fni = string(fn(i));
        field = structure.(fni);
        assignin('caller', fni, field);
    end
end