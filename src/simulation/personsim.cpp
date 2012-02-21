#include "personsim.h"
#include <iostream>
#include "gl/gltools.h"
//#include <GL/glut.h>
#include "../world/cylindricalpart.h"
#include "../world/spherepart.h"
#include "world/world.h"

namespace mr
{
IMPLEMENT_MR_OBJECT(PersonSim)

PersonSim::PersonSim()
{
	speed=rotSpeed=1.5;
	radius=0.3;
	height=1.8;
	//las ruedas no se pueden a�adir hasta no tener un mecanismo de exclusi�n de detecci�n
	CylindricalPart *wheel1=new CylindricalPart(height*2.0/3.0, radius);
	wheel1->setColor(1.0,0.1,0.1);
	wheel1->setRelativeOrientation(Z_AXIS,-PI/2);

	SpherePart *head=new SpherePart(height/3.0);
	head->setRelativePosition(Vector3D(0,0,2));
	head->setColor(1.0,0.1,0.1);

	(*this)+=wheel1;
	(*this)+=head;


}
void PersonSim::writeToStream(Stream& stream)
{
	
}
void PersonSim::readFromStream(Stream& stream)
{
	
}
void PersonSim::drawGL()
{
	//it si possible to make the drawing independent of the geometric model
	//so we overwrite the composedentity drawing function with this one
	ComposedEntity::drawGL();
	return;
}
bool PersonSim::computeGroundedLocation(Transformation3D &p)
{
//partiendo de p, la modifica para que sea una posici�n geom�tricamente v�lida, suponiendo una
//gravedad, pero sin consideraciones din�micas
	cout<<"compute ground:  "<<p<<endl;
	int i;
	
	//absolute center of the four wheels
	Vector3D abswheels;
	double dw=10.0;
	bool bw;
	//retrieving the needed data
//	Transformation3D t=getAbsoluteT3D(); //current location
	abswheels=p.position; 
	Vector3D uz=(p.getVectorW())*(-1.0);
	World *world=getWorld();

	if(world){
		//desactivo el calculo de intersecciones con mi modelo, para que no vea mis propias piezas
		setIntersectable(false);
		bw=world->rayIntersection(abswheels,uz,dw);
		setIntersectable();
		//ya tengo calculados los puntos de contacto. descarto los que superan una rueda
		if(dw>2*radius)bw=false;


		//calculo los punto de contacto te�ricos cayendo en z:
		Vector3D contact=abswheels+uz*dw;
		
		//ahora los tres puntos estan obtenidos: lo transformamos en un sdr coherente con la plataforma
	//	bool flag[4]={false,false,false,false};
	//	for(i=0;i<3;i++)flag[ord[i]]=true;
		Vector3D vu,vv,vw,npos;
		//nueva posici�n:
		cout<<"The new pos is: "<<contact<<endl;
		npos=contact;
		p.position=npos;
		//p.orientation=OrientationMatrix(vu,vv);
	}
	return true;
}
void PersonSim::simulate(double delta_t)
{
//	cout<<"simulating: "<<speed<<" "<<rotSpeed<<" "<<delta_t<<endl;
	//aqui es necesario recoger el mundo al que pertenece el world
	double delta_x=speed*delta_t;
	double delta_y=0;
	double delta_th=rotSpeed*delta_t;
	
	/*verificar si la posicion de destino es valida: se verifica
	de manera simple lanzando unos rayos  abajo segun el eje Z desde los centros de las ruedas
	para ello hay que discretizar la trayectoria en trozitos del tama�o de un radio como maximo
	normalemnete el tiempo de simulaci�n es mucho menor
	Se obtiene entonces cuatro alturas. De estas se utilizan las tres m�s estables segun un modelo din�mico
	sencillo. Adem�s se verifica que el salto dado por las ruedas de apoyo no supera su diametro .
	en cuyo caso se actualiza la posici�n y la pose. La pose se actualiza con el movimiento lineal,
	mientras que la posici�n puede ser 3d (subir una rampa por ejemplo
	*/
	Transformation3D position=getAbsoluteT3D();
	Transformation3D delta(delta_x*cos(delta_th),delta_x*sin(delta_th),0,0,0,delta_th);
	//the new posible absolute position is:
	Transformation3D newposition=position*delta;

	//el calculo de posicion de destino debera ir en una funcion para poder ser reusado
	//base del planificador RRT:
//	if(computeGroundedLocation(newposition)==false)return;
	//actualizamos la posici�n
	setAbsoluteT3D(newposition);
	//verifico que no hay colisi�n (aunque habr�a que desactivar la deteccion con las ruedas)
	World *world=getWorld();
	if(world){
		if(world->checkCollisionWith(*this)){
			setAbsoluteT3D(position); //no muevo el robot
			rotSpeed=rand()/(float)RAND_MAX;
			speed=-speed;
			return ;
		}
	}
	//esta es la posicion te�rica simple de los encoders en caso de que el robot pueda moverse
//	pose=pose*Pose(delta_x,delta_y,delta_th);

	int r=rand()%75;
	if(r==0)
	{
		rotSpeed=rand()/(float)RAND_MAX;
		speed=rand()/(float)RAND_MAX;
	}
	else if(r==1)
		rotSpeed=0;
	else if(r==2)
		speed=0;

}


}; //Namespace mr