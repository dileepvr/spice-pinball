NAME="scoreboard"

mv $NAME-Front.gtl $NAME.GTL
mv $NAME-Back.gbl $NAME.GBL

mv $NAME-Mask_Front.gts $NAME.GTS
mv $NAME-Mask_Back.gbs $NAME.GBS

mv $NAME-SilkS_Front.gto $NAME.GTO
mv $NAME-SilkS_Back.gbo $NAME.GBO

mv $NAME-PCB_Edges.gbr $NAME.GKO

mv $NAME.drl $NAME.XLN
rm $NAME-drl.pho

cp $NAME.XLN $NAME.GKO $NAME.GBO $NAME.GTO $NAME.GBS $NAME.GTS $NAME.GBL $NAME.GTL ./tosend






