# TP2 - Dual Contouring (Clément POTIN, M2 IMAGINE)


## ICP

INIT:
1 - Calculer le centroide des formes
2 - Translation source sur target					| Calcul d'initialisation (guess n°1)
3 - Appliquer une rotation aléatoire ou Identité	| Possibilité d'ACP, mais rotation aléatoire plus simple pour commencer

L'INIT sert à réaliser plusieurs ICP sur des configurations différents afin d'essayer de trouver de meilleurs résultats (pour ne pas rester bloquer dans un minimum local d'une ICP).

ALGO ICP:
4 - Calculer la matrice S = P.Q^T (^T = transpose)
[p0x, ..., pix, ..., pnx]
[p0y, ..., piy, ..., pny] Produit matriciel avec Q mis dans ce sens ↓
[p0z, ..., piz, ..., pnz]
Coordonnées des points p - coordonnées du centroïde de la forme P
Somme i=0→n `pix*qix`
S.setRotation
Regarder les distances dans le KdTree, associer les points P à leur plus proche dans Q

? - Boucle pour aller chercher les appareillements