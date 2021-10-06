# TP2 - Dual Contouring (Clément POTIN, M2 IMAGINE)


## Résumé

Pour compiler : `make clean ; make`
Pour lancer : `./tp <taille de la grille>`

Si seulement dual contouring : commencer les lignes 778 à 782.
Lancer avec des tailles de grilles comprises entre 16 et 128.

Si utilisation de la subdivision, décommenter les lignes 778 à 782.
Lancer avec une taille de grille de 8.
Chaque subdivision sera affichée 1s, dans une boucle de 6s.


## IMPORTANT

Pour lancer : `make clean ; make ; ./tp <taille de la grille>`

ATTENTION : choisir une grille de petite taille (ex: 8, la taille par défaut).
J'ai appliqué des subdivisions de la mesh obtenu avec cette grille en 8x8x8 par la suite, donc la mesh gagne en précision.

ATTENTION 2 : si ordinateur moyennement puissant, lire tout le README avant de lancer...


## Dual contouring

J'ai implémenté le dual contouring dans la fonction du même nom, l. 493 à 662.

Elle prend en entrée une taille de grille ainsi que les variables classiques demandées par la fonction HPSS.
Si `gridSize = 8`, on obtient une grille de 8x8x8, centrée sur le modèle et dont les extrémités sont celles de la bouding box de notre modèle (jusqu'à l. 538).

Les points sont parcourus et on ascocie à chaque point un booléen true/false en fonction de s'il est à l'intérieur ou à l'extérieur du modèle. (jusqu'à l. 554).

On peut ensuite facilement trouver les points de la grille "à la surface" du modèle : si l'un des sommets du cube ABCDEFGH est à l'intérieur (true) et un autre à l'extérieur (false), on ajoute un point au nuage de points constituant la surface. Les coordonnées de ce point sont celles du centre du cube (jusqu'à l. 591).

On peut également se servir de ces informations (true/false) pour parcourir les arêtes de notre grille de point. Si un côté de l'arête est à l'intérieur du modèle (true), et l'autre à l'extérieur (false), on sait qu'il faut créer une face qui fera partie de notre maillage. Cette face est créée comme sur la slide 38/44 du cours sur le dual contouring : on prend les centres des 4 cellules (cubes) adjacentes à l'arête, et on crée 2 triangles avec ces points, de façon à créer un carré.
En parcourant toutes les arêtes de la grille, on obtient un maillage "cubique" (car uniquement composé de face carrées et orientées dans le sens des axes x/y, x/z ou y/z).
On peut ensuite projeter chacun des points du maillage sur le modèle, en utilisant la fonction HPSS. Cela permet d'affiner les détails de notre mesh et retire son effet "cubique" (l. 647) (jusqu'à l. 656).

On peut donc dire que le centre des cellules projetés sur le modèle donnent un rendu de bien meilleure qualité.

### IMPORTANT :

Pour ne tester que cette partie du code, commenter les lignes 778 à 782, recompiler et lancer (`make clean ; make ; ./tp <taille de la grille>`).

Taille recommandée : entre 16 et 128 (en dessous : trop peu de détails, au dessus : trop de points).


## Subdivision de la mesh

Une fois la mesh obtenue (après dual contouring), on peut améliorer sa qualité. Pour se faire, on subdivise chaque triangle de la mesh en 4 en créant 3 nouveaux sommets au centre des arêtes du triangle. Ces nouveaux sommets sont projetés sur le modèle (sinon il n'y aurait aucune différence), et on recrée la mesh avec ces nouveaux triangles (fonction subdivideMeshVertices, l. 664 à 705).

Le temps de subdivision est par contre très faible par rapport à la création d'une mesh avec une grille de grande taille. J'ai donc pu créer un maillage à partir d'une grille de 8x8x8, et le subdiviser 5 fois de façon très rapide, ce qui m'a permis d'obtenir des résultats similaires à ceux obtenus avec une grille de 128x128x128.

### IMPORTANT :

Pour tester la partie subdivision, décommenter les lignes 778 à <?>. Chaque ligne décommentée va créer une nouvelle mesh subdivisée par rapport à la précédente. J'ai pour ma part pu faire tourner le TP avec les lignes 778 à 782 décommentées (sur un ordinateur portable), mais des PC moins puissants pourraient avoir du mal.
Recompiler et lancer (`make clean ; make ; ./tp <taille de la grille>`).

Considérez que chaque subdivison revient presque à doubler la taille de la grille, en terme de poids du rendu. Donc si grille de taille 8 et 5 subdivisions, on obtient une mesh à peu près aussi lourde à afficher qu'une mesh réalisée à partir d'une grille de taille 128 (8x2^4, selon le ressenti).

Chaque mesh subdivisée est indépendante, et affichée 1s de la plus basse résolution à la plus haute résolution disponible (boucles de 6s). Je conseille donc d'essayer de décommenter toutes les lignes, mais je laisse des captures d'écran et gifs de mon travail dans le cas où cela serait trop lourd à lancer (aucun problème pour ma part).