
classdef Fields
    properties
        field_range
        field_size
        field_dim
        xmin
        xmax
        ymin
        ymax
        zmin
        zmax
    end
    
    methods
        function obj = Fields(range)
            obj.field_range = range;
            obj.field_dim = size(range,1);
            obj.field_size = range(:,2)-range(:,1);
        end
        function field_x = getFieldRangeX(obj,i)
            if exist('i','var')
                field_x = obj.field_range(1,i);
            else
                field_x = obj.field_range(1,:);
            end
        end
        function field_y = getFieldRangeY(obj,i)
            if exist('i','var')
                field_y = obj.field_range(2,i);
            else
                field_y = obj.field_range(2,:);
            end
        end
        function field_z = getFieldRangeZ(obj,i)
            if exist('i','var')
                field_z = obj.field_range(2,i);
            else
                field_z = obj.field_range(2,:);
            end
        end
        function field_size = getFieldSize(obj)
            field_size = obj.field_size;
        end
    end
end