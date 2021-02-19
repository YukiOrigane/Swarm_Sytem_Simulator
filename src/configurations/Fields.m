
classdef Fields
    properties
        field_range
        field_size
        field_dim
    end
    
    methods
        function obj = Fields(range)
            obj.field_range = range;
            obj.field_dim = size(range,1);
            obj.field_size = range(:,2)-range(:,1);
        end
        function field_x = getFieldRangeX(obj)
            field_x = obj.field_range(1,:);
        end
        function field_y = getFieldRangeY(obj)
            field_y = obj.field_range(1,:);
        end
        function field_y = getFieldRangeZ(obj)
            field_y = obj.field_range(1,:);
        end
        function field_size = getFieldSize(obj)
            field_size = obj.field_size;
        end
    end
end