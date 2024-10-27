# balance_hx711


git add .;git commit -m "main code";git push -u -f origin main


git init
git lfs install

git config --global user.name "Jacques Lagnel"
git config --global user.email "jacques.lagnel@inrae.fr"
git remote add origin git@forgemia.inra.fr:Jacques.Lagnel/thaliadb.git

git lfs track "thalia.tar.gz"

git add .gitattributes
git add .

commit -m "Initial lfs"
git push -u origin master


# and after only

git add .
commit -m "update"

Il faut exec cette cmd pur pouvoir créer un nouveau repos sur le github

Sinon il faut obligatoirement le crèer sur github avant 
curl -H "Authorization: token YOUR_GITHUB_TOKEN" \
     -d '{"name": "monprojet"}' \
     https://api.github.com/user/repos

git remote add origin git@github.com:votre_utilisateur/monprojet.git


git push -u origin master






20:49:08.162 -> Copyright @2019-2020 Heltec Automation.All rights reserved.
20:49:17.982 -> INIT rtc RAM offset: 164264
20:49:22.228 -> ADC-offset:	164264	Vbat-offset:	3428	Tempint-offset:	23	TempExt-offset:	22	TempInt:	23	TempExt:	22
20:49:22.260 -> 	Vbat:	3428	poids ori:	0.0	0	poids cor:	0.0	0
20:49:22.260 -> 
20:49:22.260 -> AT Rev 1.3
20:49:22.260 -> +AutoLPM=1
20:49:22.260 -> 
20:49:22.260 -> +LORAWAN=1
20:49:22.260 -> 
20:49:22.260 -> +KeepNet=1
20:49:22.260 -> +OTAA=1
20:49:22.260 -> +Class=A
20:49:22.260 -> +ADR=1
20:49:22.260 -> +IsTxConfirmed=0
20:49:22.260 -> +AppPort=2
20:49:22.260 -> +DutyCycle=30000
20:49:22.260 -> +ConfirmedNbTrials=4
20:49:22.260 -> +ChMask=0000000000000000000000FF
20:49:22.260 -> +DevEui=2232330000888802(For OTAA Mode)
20:49:22.260 -> +AppEui=0000000000000000(For OTAA Mode)
20:49:22.292 -> +AppKey=88888888888888888888888888886601(For OTAA Mode)
20:49:22.293 -> +NwkSKey=15B1D0EFA463DFBE3D11181E1EC7DA85(For ABP Mode)
20:49:22.293 -> +AppSKey=D72C78758CDCCABF55EE4A778D16EF67(For ABP Mode)
20:49:22.293 -> +DevAddr=007E6AE1(For ABP Mode)
20:49:22.293 -> 
20:49:22.422 -> 
20:49:22.422 -> LoRaWAN EU868 Class A start!
20:49:22.422 -> 
20:49:22.552 -> joining...joined
20:49:32.204 -> Timout:	60	Fault:	0	hx711:	0	TempInt:	23	TempExt:	22	Vbat:	3428
20:49:32.204 -> unconfirmed uplink sending ...
20:49:37.587 -> received unconfirmed downlink: rssi = -96, snr = 4, datarate = 3
20:50:36.667 -> Timout:	60	Fault:	0	hx711:	0	TempInt:	23	TempExt:	22	Vbat:	3428
20:50:36.667 -> unconfirmed uplink sending ...
20:50:42.028 -> received unconfirmed downlink: rssi = -93, snr = 6, datarate = 3
20:51:41.037 -> Timout:	60	Fault:	0	hx711:	0	TempInt:	22	TempExt:	22	Vbat:	3428
20:51:41.086 -> unconfirmed uplink sending ...
