function enregistrement()

    % Création de l'objet vidéo
    videoFile = 'animation_bateau.mp4';  % Nom du fichier
    v = VideoWriter(videoFile, 'MPEG-4');  % Choix du format MP4 (ou 'Motion JPEG AVI' pour AVI)
    v.FrameRate = 30;  % Définir la fréquence d'images
    open(v);  % Ouvrir l'objet vidéo
    
    % Création de la figure
    figure(600);
    set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1])
    hold on
    axis equal
    grid on
    
    % Initialisation des courbes
    h1 = plot(nan, nan, 'k', 'LineWidth', 2); % GPS
    h2 = plot(nan, nan, '--r', 'LineWidth', 2); % Reconstruit
    h3 = plot(nan, nan, 'b', 'LineWidth', 6); % Bateau
    h4 = quiver(0, 0, 0, 0, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Vitesse longitudinale
    h5 = quiver(0, 0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Vitesse latérale
    h6 = quiver(0, 0, 0, 0, 0, '--k', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Cap
    
    legend({'GPS', 'Reconstruit', 'Bateau', 'Vitesse longitudinale', 'Vitesse latérale','Cap'}, 'FontSize', 15);
    title('Trajectoire du bateau', 'FontSize', 20);
    xlabel('Y en m', 'FontSize', 15);
    ylabel('X en m', 'FontSize', 15);
    
    for i = 1:length(t)
        % Mise à jour des données
        set(h1, 'XData', ym(1:length(t)), 'YData', xm(1:length(t)));
        set(h2, 'XData', y_cal, 'YData', x_cal);
        
        % Calcul des positions du bateau
        bat = [-L/2, 0, L/2];
        x_bat = x_cal(i) + bat .* cos(psi_est_filt(i) + beta_est_filt(i));
        y_bat = y_cal(i) + bat .* sin(psi_est_filt(i) + beta_est_filt(i));
        set(h3, 'XData', y_bat, 'YData', x_bat);
        
        % Mise à jour des vecteurs de vitesse
        set(h4, 'XData', y_cal(i), 'YData', x_cal(i), ...
            'UData', 50 * (u_est(i) + bu_est(i)) * sin(psi_est_filt(i) + beta_est_filt(i)), ...
            'VData', 50 * (u_est(i) + bu_est(i)) * cos(psi_est_filt(i) + beta_est_filt(i)));
    
        set(h5, 'XData', y_cal(i), 'YData', x_cal(i), ...
            'UData', 50 * (v_est_filt(i) + bv_est(i)) * cos(psi_est_filt(i) + beta_est_filt(i)), ...
            'VData', -50 * (v_est_filt(i) + bv_est(i)) * sin(psi_est_filt(i) + beta_est_filt(i)));
    
        set(h6, 'XData', y_cal(i), 'YData', x_cal(i), ...
            'UData', 100 * sin(psi_est_filt(i)), ...
            'VData', 100 * cos(psi_est_filt(i)));
        
        xlim([y_cal(i) - 200, y_cal(i) + 200]);
        ylim([x_cal(i) - 200, x_cal(i) + 200]);
    
        drawnow;
        
        % Capture et enregistrement du frame
        frame = getframe(gcf);  % Capture du graphique
        writeVideo(v, frame);   % Écriture de l'image dans la vidéo
    end
    
    % Fermer et sauvegarder la vidéo
    close(v);
    disp('Vidéo enregistrée avec succès !');

end