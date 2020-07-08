#include "Interp.h"

#include <g3log/g3log.hpp>

using namespace g3; 

namespace interp {

int index = 0;

Interp::Interp(coords::VecIdent* c, domain::VecIdent *d) 
  : coords_(c), type_(dom_vecIdent_type), ident_(d) {
}

Interp::Interp(coords::VecExpr *c, domain::VecExpr *d) 
  : coords_(c), type_(dom_vecExpr_type), expr_(d)  {
}

Interp::Interp(coords::Vector *c, domain::Vector *d)
  : coords_(c), type_(dom_vector_type), vector_(d) {
}

Interp::Interp(coords::Vector_Def *c, domain::Vector_Def *d) 
  : coords_(c), type_(dom_vector_def_type), def_(d) {
}

Interp::Interp(coords::Vector_Assign *c, domain::Vector_Assign *d) 
  : coords_(c), type_(dom_vector_assign_type), assign_(d) {
}


Interp::Interp(coords::ScalarIdent* c, domain::ScalarIdent *d) 
  : coords_(c), type_(dom_floatIdent_type), float_ident_(d) {
}

Interp::Interp(coords::ScalarExpr *c, domain::ScalarExpr *d) 
  : coords_(c), type_(dom_floatExpr_type), float_expr_(d)  {
}

Interp::Interp(coords::Scalar *c, domain::Scalar *d)
  : coords_(c), type_(dom_float_type), float_(d) {
}

Interp::Interp(coords::Scalar_Def *c, domain::Scalar_Def *d) 
  : coords_(c), type_(dom_float_def_type), float_def_(d) {
}

Interp::Interp(coords::Scalar_Assign *c, domain::Scalar_Assign *d) 
  : coords_(c), type_(dom_float_assign_type), float_assign_(d) {
}

Interp::Interp(coords::TransformIdent* c, domain::TransformIdent *d) 
  : coords_(c), type_(dom_transformIdent_type), transform_ident_(d) {
}

Interp::Interp(coords::TransformExpr *c, domain::TransformExpr *d) 
  : coords_(c), type_(dom_transformExpr_type), transform_expr_(d)  {
}

Interp::Interp(coords::Transform *c, domain::Transform *d)
  : coords_(c), type_(dom_transform_type), transform_(d) {
}

Interp::Interp(coords::Transform_Def *c, domain::Transform_Def *d) 
  : coords_(c), type_(dom_transform_def_type), transform_def_(d) {
}

Interp::Interp(coords::Transform_Assign *c, domain::Transform_Assign *d) 
  : coords_(c), type_(dom_transform_assign_type), transform_assign_(d) {
}

/**********
 * Abstract
 **********/


std::string Interp::toString() const {
  LOG(FATAL) << "Interp::toString: Error. Should not be called. Abstract.\n";
  return "";
}

/******
 * Ident
 ******/

VecIdent::VecIdent(coords::VecIdent* c, domain::VecIdent* d) : Interp(c,d) {
}

std::string VecIdent::toString() const {
  std::string ret = "";
//  ret += "( ";
  ret += "def ";
  ret += coords_->toString() + "_var";
  ret += " : @peirce.vector_variable " + ident_->getSpaceContainer()->toString();
  ret += " := @peirce.vector_variable.mk ";
  ret += ident_->getSpaceContainer()->toString() + " " + std::to_string(++index);
//  ret += " )";
  return ret; 
}

ScalarIdent::ScalarIdent(coords::ScalarIdent* c, domain::ScalarIdent* d) : Interp(c,d) {
}

std::string ScalarIdent::toString() const {
  std::string ret = "";
  ret += "def ";
  ret += coords_->toString() + "_var";
  ret += " : peirce.scalar_variable := peirce.scalar_variable.mk ";
  ret += /*float_ident_->getSpaceContainer()->toString() +*/ " " + std::to_string(++index);
  //ret += float_ident_->getSpaceContainer()->toString();
//  ret += " )";
  return ret;
}

TransformIdent::TransformIdent(coords::TransformIdent* c, domain::TransformIdent* d) : Interp(c,d) {
}

std::string TransformIdent::toString() const {
  std::string ret = "";
  ret += "def ";
  ret += coords_->toString() + "_var";
  ret += " : @peirce.transform_variable " + transform_ident_->getSpaceContainer()->toString();
  ret += " := @peirce.transform_variable.mk ";
  ret += transform_ident_->getSpaceContainer()->toString() + " " + std::to_string(++index);
//  ret += " )";
  return ret;
}


/*****
 * Expr
 *****/

VecExpr::VecExpr(coords::VecExpr* c, domain::VecExpr* d)  : Interp(c, d)  {
}

std::string VecExpr::toString() const {
  LOG(FATAL) << "Error. Call to abstract interp::VecIdent::toString().\n";
  return "Should not call abstract interp::VecExpr::toString().";
}

ScalarExpr::ScalarExpr(coords::ScalarExpr* c, domain::ScalarExpr* d)  : Interp(c, d)  {
}

std::string ScalarExpr::toString() const {
  LOG(FATAL) << "Error. Call to abstract interp::ScalarIdent::toString().\n";
  return "Should not call abstract interp::ScalarIdent::toString().";
}

TransformExpr::TransformExpr(coords::TransformExpr* c, domain::TransformExpr* d)  : Interp(c, d)  {
}

std::string TransformExpr::toString() const {
  LOG(FATAL) << "Error. Call to abstract interp::TransformIdent::toString().\n";
  return "Should not call abstract interp::TransformIdent::toString().";
}


VecVarExpr::VecVarExpr(coords::VecVarExpr* c, domain::VecVarExpr* d) : VecExpr(c, d) {
}

std::string VecVarExpr::toString() const {
  std::string ret = "";
  ret += "( @peirce.vector_expression.vector_var " + expr_->getSpaceContainer()->toString() + " ";
  ret += coords_->toString() + "_var";
  ret += " : peirce.vector_expression ";
  ret += expr_->getSpaceContainer()->toString(); 
  ret += " ) ";
  return ret;
}

ScalarVarExpr::ScalarVarExpr(coords::ScalarVarExpr* c, domain::ScalarVarExpr* d) :ScalarExpr(c, d) {
}

std::string ScalarVarExpr::toString() const {
  std::string ret = "";
  ret += "( peirce.scalar_expression.scalar_var ";
  ret += coords_->toString() + "_var";
  ret += " : peirce.scalar_expression ";
 // ret += float_expr_->getSpaceContainer()->toString(); 
  ret += " ) ";//: peirce.scalar ";
  return ret;
}

TransformVarExpr::TransformVarExpr(coords::TransformVarExpr* c, domain::TransformVarExpr* d) :TransformExpr(c, d) {
}

std::string TransformVarExpr::toString() const {
  std::string ret = "";
  ret += "( @peirce.transform_expression.transform_var " + transform_expr_->getSpaceContainer()->toString() + " ";
  ret += coords_->toString() + "_var";
  ret += " : peirce.transform_expression ";
  ret += transform_expr_->getSpaceContainer()->toString(); 
  ret += " ) ";//: peirce.Transform ";
  return ret;
}




VecVecAddExpr::VecVecAddExpr(coords::VecVecAddExpr* c, domain::VecVecAddExpr* d, 
                             interp::Interp *mem, interp::Interp *arg)  
  : VecExpr(c, d), mem_(mem), arg_(arg) {
}

 
std::string VecVecAddExpr::toString() const {
  std::string ret = "";
  ret += "( @peirce.vector_expression.vector_add " + expr_->getSpaceContainer()->toString() + " ";
  ret += mem_->toString();
  ret += " ";
  ret += arg_->toString();
  ret += " : peirce.vector_expression ";
  ret += expr_->getSpaceContainer()->toString(); 
  ret += " )";
  return ret;  
} 

VecScalarMulExpr::VecScalarMulExpr(coords::VecScalarMulExpr* c, domain::VecScalarMulExpr* d, 
                             interp::Interp *vec, interp::Interp *flt)  
  : VecExpr(c, d), vec_(vec), flt_(flt) {
}

 
std::string VecScalarMulExpr::toString() const {
  std::string ret = "";
  ret += "( @peirce.vector_expression.scalar_vector_mul " + expr_->getSpaceContainer()->toString() + " ";
  ret += flt_->toString();
  ret += " ";
  ret += vec_->toString();
  ret += " : peirce.vector_expression ";
  ret += expr_->getSpaceContainer()->toString(); 
  ret += " )";
  return ret;  
} 

TransformVecApplyExpr::TransformVecApplyExpr(coords::TransformVecApplyExpr * c, domain::TransformVecApplyExpr *d,
                interp::Interp *tfm, interp::Interp *vec)  
  : VecExpr(c, d), tfm_(tfm), vec_(vec) {
}

 
std::string TransformVecApplyExpr::toString() const {
  std::string ret = "";
  ret += "( peirce.transform_apply ";
  ret += tfm_->toString();
  ret += " ";
  ret += vec_->toString();
  ret += " : peirce.vector_expression ";
  ret += expr_->getSpaceContainer()->toString(); 
  ret += " )";
  return ret;  
} 

ScalarScalarAddExpr::ScalarScalarAddExpr(coords::ScalarScalarAddExpr* c, domain::ScalarScalarAddExpr* d, 
                             interp::Interp *lhs, interp::Interp *rhs)  
  : ScalarExpr(c, d), lhs_(lhs), rhs_(rhs) {
}

 
std::string ScalarScalarAddExpr::toString() const {
  std::string ret = "";
  ret += "( peirce.scalar_expression.scalar_add ";
  ret += lhs_->toString();
  ret += " ";
  ret += rhs_->toString();
 // ret += " : peirce.scalar ";
  //ret += float_expr_->getSpaceContainer()->toString(); 
  ret += " : peirce.scalar_expression ) ";
  return ret;  
} 

ScalarScalarMulExpr::ScalarScalarMulExpr(coords::ScalarScalarMulExpr* c, domain::ScalarScalarMulExpr* d, 
                             interp::Interp *lhs, interp::Interp *rhs)  
  : ScalarExpr(c, d), lhs_(lhs), rhs_(rhs) {
}

 
std::string ScalarScalarMulExpr::toString() const {
  std::string ret = "";
  ret += "( peirce.scalar_expression.scalar_mul ";
  ret += lhs_->toString();
  ret += " ";
  ret += rhs_->toString();
  //ret += " : peirce.scalar ";
 // ret += float_expr_->getSpaceContainer()->toString(); 
  ret += " : peirce.scalar_expression ) ";
  return ret;  
} 

TransformTransformComposeExpr::TransformTransformComposeExpr(coords::TransformTransformComposeExpr *c, domain::TransformTransformComposeExpr *d,
                interp::Interp *outer, interp::Interp *inner) 
  : TransformExpr(c, d), outer_(outer), inner_(inner) {
}

 
std::string TransformTransformComposeExpr::toString() const {
  std::string ret = "";
  ret += "( peirce.transform_compose ";
  ret += outer_->toString();
  ret += " ";
  ret += inner_->toString();
  ret += " : peirce.transform_expression ";
  ret += transform_expr_->getSpaceContainer()->toString() + " ) "; 
  return ret;  
} 


VecParenExpr::VecParenExpr
    (coords::VecParenExpr* c, domain::VecParenExpr* d, interp::VecExpr *e) 
    : VecExpr(c, d), paren_expr_(e) {
}

std::string VecParenExpr::toString() const {
  std::string ret = "";
  ret += "( @peirce.vector_expression.vector_paren " + expr_->getSpaceContainer()->toString() + " ";
  ret += paren_expr_->toString();
  ret += " : peirce.vector_expression ";

  // TODO: Abstract superclass data members
  ret += expr_->getSpaceContainer()->toString(); 

  ret += " ) ";
  return ret;  
} 


ScalarParenExpr::ScalarParenExpr
    (coords::ScalarParenExpr* c, domain::ScalarParenExpr* d, interp::ScalarExpr *e) 
    : ScalarExpr(c, d), paren_expr_(e) {
}

std::string ScalarParenExpr::toString() const {
  std::string ret = "";
  ret += "( peirce.scalar_expression.scalar_paren ";
  ret += paren_expr_->toString();
  ret += " : peirce.scalar_expression ) ";

  // TODO: Abstract superclass data members
 // ret += float_expr_->getSpaceContainer()->toString(); 

  ret += " ";
  return ret;  
} 


TransformParenExpr::TransformParenExpr
    (coords::TransformParenExpr* c, domain::TransformParenExpr* d, interp::TransformExpr *e) 
    : TransformExpr(c, d), paren_expr_(e) {
}

std::string TransformParenExpr::toString() const {
  std::string ret = "";
  ret += "( @peirce.transform_expression.transform_paren " + transform_expr_->getSpaceContainer()->toString() + " ";
  ret += paren_expr_->toString();
  ret += " : peirce.transform_expression ";

  // TODO: Abstract superclass data members
  ret += transform_expr_->getSpaceContainer()->toString(); 

  ret += " )";
  return ret;  
} 




/*******
* Vector
********/
 
Vector::Vector(coords::Vector* c, domain::Vector* d) : Interp(c, d) {}

std::string Vector::toString() const {
  LOG(INFO) << "Interp::Vector::toString().\n";
  return "A_Vector";
}
 
Scalar::Scalar(coords::Scalar* c, domain::Scalar* d) : Interp(c, d) {}

std::string Scalar::toString() const {
  LOG(INFO) << "Interp::Scalar::toString().\n";
  return "A_Scalar";
}
 
Transform::Transform(coords::Transform* c, domain::Transform* d) : Interp(c, d) {}

std::string Transform::toString() const {
  LOG(INFO) << "Interp::Transform::toString().\n";
  return "A_Transform";
}

Vector_Lit::Vector_Lit(coords::Vector_Lit* c, domain::Vector_Lit* d) : Vector(c,d) {
}

std::string Vector_Lit::toString() const {
  std::string ret = "";
  ret += " ( @peirce.vector_expression.vector_literal " + vector_->getSpaceContainer()->toString() + " ";
  ret += " ( @peirce.vector.mk " + vector_->getSpaceContainer()->toString() + " ";
  ret += " ";
  ret += static_cast<coords::Vector_Lit *>(coords_)->toString();
  ret += "  : peirce.vector " + vector_->getSpaceContainer()->toString();
  ret += " ) : peirce.vector_expression " + vector_->getSpaceContainer()->toString() + " ) ";
  return ret;
}

Scalar_Lit::Scalar_Lit(coords::Scalar_Lit* c, domain::Scalar_Lit* d) : Scalar(c,d) {
}

std::string Scalar_Lit::toString() const {
  std::string ret = "";
  ret += "(";
//  ret += float_->getSpaceContainer()->toString();
  ret += " peirce.scalar_expression.scalar_lit ";
  ret += static_cast<coords::Scalar_Lit *>(coords_)->toString();
  ret += " : peirce.scalar_expression )";
  return ret;
}

Transform_Lit::Transform_Lit(coords::Transform_Lit* c, domain::Transform_Lit* d, interp::Interp * arg1, interp::Interp * arg2, interp::Interp * arg3) : Transform(c,d),
arg1_{arg1}, arg2_{arg2}, arg3_{arg3} {
}

std::string Transform_Lit::toString() const {
  std::string ret = "";
  ret += "( @peirce.transform_expression.transform_literal " + transform_->getSpaceContainer()->toString() + " ";
  ret += " ( @peirce.transform.mk " + transform_->getSpaceContainer()->toString() + " ";
  ret += " ";
  ret += arg1_->toString();
  ret += " ";
  ret += arg2_->toString();
  ret += " ";
  ret += arg3_->toString();
  ret += " : peirce.transform " + transform_->getSpaceContainer()->toString() + " ) : peirce.transform_expression " + transform_->getSpaceContainer()->toString() + " )";
  return ret;
}


Vector_Var::Vector_Var(coords::Vector_Var* c, domain::Vector_Var* d) : Vector(c,d) {

}

std::string Vector_Var::toString() const {
  LOG(FATAL) << "interp::Vector_Var::toString. Error. Not implemented.\n";
  return "";
}

Scalar_Var::Scalar_Var(coords::Scalar_Var* c, domain::Scalar_Var* d) : Scalar(c,d) {

}

std::string Scalar_Var::toString() const {
  LOG(FATAL) << "interp::Vector_Var::toString. Error. Not implemented.\n";
  return "";
}

Transform_Var::Transform_Var(coords::Transform_Var* c, domain::Transform_Var* d) : Transform(c,d) {

}

std::string Transform_Var::toString() const {
  LOG(FATAL) << "interp::Vector_Var::toString. Error. Not implemented.\n";
  return "";
}


Vector_Expr::Vector_Expr(coords::Vector_Expr *c, domain::Vector_Expr* d, interp::Interp *expr_interp) 
  : Vector(c,d), expr_interp_(expr_interp) {

}

std::string Vector_Expr::toString() const {
  return getVector_Expr()->toString(); 
}


Scalar_Expr::Scalar_Expr(coords::Scalar_Expr *c, domain::Scalar_Expr* d, interp::ScalarExpr *expr_interp) 
  : Scalar(c,d), expr_interp_(expr_interp) {

}

std::string Scalar_Expr::toString() const {
  return getScalar_Expr()->toString(); 
}


Transform_Expr::Transform_Expr(coords::Transform_Expr *c, domain::Transform_Expr* d, interp::TransformExpr *expr_interp) 
  : Transform(c,d), expr_interp_(expr_interp) {

}

std::string Transform_Expr::toString() const {
  return getTransform_Expr()->toString(); 
}


/****
 * Def
 ****/

Vector_Def::Vector_Def(coords::Vector_Def* c, domain::Vector_Def* d, interp::VecIdent *id, interp::Interp *vec) 
  : Interp(c,d), id_(id), vec_(vec) { 
}
/*


*/

std::string Vector_Def::toString() const {
  std::string ret = "";
  ret += id_->toString();
  ret += "\n";
  try{//why is tthis try catch here? incase there is no rhs...?
    //if(vec_)
    //  ret += vec_->toString(); 
    ret += "def " + id_->coords_->toString();
    ret += " : peirce.vector_cmd := @peirce.vector_cmd.assmt " + id_->ident_->getSpaceContainer()->toString() + " " + id_->coords_->toString() + "_var" + " ( " + vec_->toString() + " ) \n";
  }
  catch(std::exception ex)
  {

  }
  return ret;
}

Scalar_Def::Scalar_Def(coords::Scalar_Def* c, domain::Scalar_Def* d, interp::ScalarIdent *id, interp::Interp *flt) 
  : Interp(c,d), id_(id), flt_(flt) { 
}
std::string Scalar_Def::toString() const {
  std::string ret = "";
  ret += id_->toString();
  ret += "\n";
  try{//why is tthis try catch here? incase there is no rhs...?
    //if(vec_)
    //  ret += vec_->toString(); 
    ret += "def " + id_->coords_->toString();
    ret += " : peirce.scalar_cmd := peirce.scalar_cmd.assmt " + id_->coords_->toString() + "_var" + " ( " + flt_->toString() + " ) \n";
  }
  catch(std::exception ex)
  {

  }
  return ret;
}

Transform_Def::Transform_Def(coords::Transform_Def* c, domain::Transform_Def* d, interp::TransformIdent *id, interp::Interp *tfm) 
  : Interp(c,d), id_(id), tfm_(tfm) { 
}
std::string Transform_Def::toString() const {
  std::string ret = "";
  ret += id_->toString();
  ret += "\n";
  try{//why is tthis try catch here? incase there is no rhs...?
    //if(vec_)
    //  ret += vec_->toString(); 
    ret += "def " + id_->coords_->toString();
    ret += " : peirce.transform_cmd := @peirce.transform_cmd.assmt " + id_->transform_ident_->getSpaceContainer()->toString() + " " + id_->coords_->toString() + "_var" + " ( " + tfm_->toString() + " ) \n";
  }
  catch(std::exception ex)
  {

  }
  return ret;
}

Vector_Assign::Vector_Assign(coords::Vector_Assign* c, domain::Vector_Assign* d, interp::VecVarExpr *id,  interp::Interp *vec) 
  : Interp(c,d), id_(id), vec_(vec) { 
}
std::string Vector_Assign::toString() const {
  std::string ret = "";//"#check ";
  //ret += id_->toString();
  //ret += " == ";
  try{
    ret += "def " + id_->coords_->toString() + "_" + std::to_string(++index);
    ret += " : peirce.vector_cmd := @peirce.vector_cmd.assmt " + id_->expr_->getSpaceContainer()->toString() + " " + id_->coords_->toString() + "_var" + " ( " + vec_->toString() + " ) \n";
  }
  catch(std::exception ex)
  {

  }
  return ret;
} 

Scalar_Assign::Scalar_Assign(coords::Scalar_Assign* c, domain::Scalar_Assign* d, interp::ScalarVarExpr *id, interp::Interp *flt) 
  : Interp(c,d), id_(id), flt_(flt) { 
}
std::string Scalar_Assign::toString() const {
  std::string ret = "";//"#check ";
  //ret += id_->toString();
  //ret += " == ";
  try{
    ret += "def " + id_->coords_->toString() + "_" + std::to_string(++index);
    ret += " : peirce.scalar_cmd := peirce.scalar_cmd.assmt " + id_->coords_->toString() + "_var" + " ( " + flt_->toString() + " ) \n";
  }
  catch(std::exception ex)
  {

  }
  return ret;
}

Transform_Assign::Transform_Assign(coords::Transform_Assign* c, domain::Transform_Assign* d, interp::TransformVarExpr *id,  interp::Interp *tfm) 
  : Interp(c,d), id_(id), tfm_(tfm) { 
}
std::string Transform_Assign::toString() const {
  std::string ret = "";//"#check ";
  //ret += id_->toString();
  //ret += " == ";
  try{
   // if(this->tfm_)
    //  ret += tfm_->toString(); 
    
    ret += "def " + id_->coords_->toString() + "_" + std::to_string(++index);
    ret += " : peirce.transform_cmd := @peirce.transform_cmd.assmt " + id_->transform_expr_->getSpaceContainer()->toString() + " " + id_->coords_->toString() + "_var" + " ( " + tfm_->toString() + " ) \n";
  }
  catch(std::exception ex)
  {

  }
  return ret;
}



} // namespace coords
