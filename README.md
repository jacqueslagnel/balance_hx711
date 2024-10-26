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


