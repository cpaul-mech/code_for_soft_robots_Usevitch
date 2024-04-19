function [  ] = Plot_Robot( Edges, x, Index, r )
%Plot the Edges in a Given Order
%     Color={'r','g','b','k'};
%     Color=[1 0 0;
%            0 1 0;
%            0 0 1;
%            1 0 1];
%     r=.05;
%     r=.075;
    %How to use the different sizes.
    for i=1:size(Edges,1)
        curve=[x(Edges(i,1),:)', x(Edges(i,2),:)'];
%         colormap(Color(Index(i),:));
        [x_plot,y_plot,z_plot]=tubeplot_C(curve,r);
%         color={r,k}
%         
%         if Index==1
%            color='r' ;
%         else
%             color='k';
%         end
%         map = [1 0 0
%                 0.1 0.1 .1];
%             colormap(map)
        surf(x_plot,y_plot,z_plot,Index(i)*ones(size(x_plot)),'edgecolor','none')
%         surf(x_plot,y_plot,z_plot,color,'edgecolor','none')
        
        %I could generate many lines and use tubeplot? 
%         plot3(x(Edges(i,:),1),x(Edges(i,:),2),x(Edges(i,:),3),Color{Index(i)})  %I could one day replace this with tube plot or something like that
        hold on
    end
    
    %Plot Spheres as the nodes
    r_s=r*1.5;
    [x_p,y_p,z_p]=sphere();
    for i=1:size(x,1)
        surf(r_s*x_p + x(i,1),r_s*y_p + x(i,2),r_s*z_p + x(i,3),ones(size(x_p))*max(max(Index)),'edgecolor','none')
    end
    
    hold off
end

