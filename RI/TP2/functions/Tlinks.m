function AA=Tlinks(DH)

AA=zeros(4,4,size(DH,1));

for k=1:size(DH,1)
    AA(:,:,k)=Tlink(DH(k,1),DH(k,2),DH(k,3),DH(k,4));
end