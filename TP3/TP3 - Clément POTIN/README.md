# TP3 - ICP (Clément POTIN, M2 IMAGINE)

## Résumé

Tout le code du TP est contenu dans le fichier `tp.cpp`.

Pour compiler : `make clean ; make`
Pour lancer : `./tp`

Lorsque le calcul de l'ICP est terminé et que les modèles sont affichés :

- Appuyer sur `a` pour activer/désactiver l'affichage de l'animation (modèle rouge) ;
- Appuyer sur `b` pour activer/désactiver l'affichage du modèle de base (modèle bleu) ;
- Appuyer sur `r` pour activer/désactiver l'affichage du modèle résultant (modèle vert).

Code modifiable :

- Pour changer les modèles chargés : lignes 656 et 657 ;
- Pour changer le nombre d'itérations de l'ICP : ligne 53 ;
- Pour changer la vitesse de l'animation (en images par seconde) : ligne 52.

Des gifs montrant les résultats de ce TP sont disponibles dans le dossier `Screenshots & Gifs`.

## Problèmes rencontrés

L'ICP tombe souvent dans un minima local. Il faut parfois une dizaine d'ICP avant d'obtenir le résultat recherché (voir minima-local.gif).
Ce constat est d'autant plus vrai lorsque les modèles ne sont pas tout à fait identique (Q partie de P, ou l'inverse).

Le problème empire encore lorsque des parties entières manquent (recalage d'une statue sans bras sur une statue avec, par exemple). Ceci est dû au fait que ces points n'existant pas dans l'autre modèle ont une influence importante sur l'ICP. On peut pour minimiser ce problème éviter d'utiliser les couples de points trop éloignés dans le calcul de rotation de l'ICP (voir **Améliorations possibles**).

## Améliorations possibles

- Lorsqu'un minima local est rencontré (score précédent ~= score courant), arrêter l'ICP ;
- Faire des rotations aléatoires au début de l'ICP, et attendre un score supérieur à un certain seul avant de commencer l'algorithme, pour limiter le nombre de minimas rencontrés ;
- Réaliser une ACP en début d'ICP afin d'essayer d'aligner les axes des deux modèles ;
- Ne sélectionner, pour la rotation, que les couples de point dont la distance est inférieure à la distance moyenne, ou à un certain seuil défini, pour que ces points n'influencent pas l'ICP ;
- Appliquer un coeficient aux couples de points en fonction de leur proximité ;
